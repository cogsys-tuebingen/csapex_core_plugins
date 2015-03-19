/// HEADER
#include "mlp.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

CSAPEX_REGISTER_CLASS(csapex::MLP, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

MLP::MLP() :
    mlp_input_size_(0),
    mlp_output_size_(0)
{
}

void MLP::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("Features");
    out_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Labeled Features");
}

void MLP::setupParameters(Parameterizable& parameters)
{
    addParameter(param::ParameterFactory::declarePath("MLP path",
                                                      param::ParameterDescription("Path to a saved MLP."),
                                                      true,
                                                      "",
                                                      "*.yaml"),
                 std::bind(&MLP::load, this));

}

namespace {
inline unsigned int maxIndex(const std::vector<double> &nn_output)
{
    double       max = std::numeric_limits<double>::min();
    unsigned int pos = 0;
    for(unsigned int i = 0 ; i < nn_output.size() ; ++i) {
        if(nn_output.at(i) > max) {
            max = nn_output.at(i);
            pos = i;
        }
    }

    return pos;
}

template<typename U, typename V>
inline void convertNumeric(const std::vector<U> &src,
                           std::vector<V> &dst)
{
    dst.resize(src.size());
    auto it_src = src.begin();
    auto it_dst = dst.begin();
    for(; it_src != src.end() ; ++it_src, ++it_dst)
        *it_dst = *it_src;

}
}


void MLP::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> in = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<FeaturesMessage> >      out(new std::vector<FeaturesMessage>());

    m_.lock();
    if(mlp_input_size_ == 0 || mlp_output_size_ == 0) {
        *out = *in;
        modifier_->setWarning("cannot classfiy, no mlp loaded");
    } else {
        out->resize(in->size());

        auto it_in  = in->begin();
        auto it_out = out->begin();

        for(; it_in != in->end() ; ++it_in, ++it_out) {
            if(it_in->value.size() != mlp_input_size_) {
                std::stringstream ss;
                ss << "Wrong feature count '" << it_in->value.size()
                   << "', expected '" << mlp_input_size_ << "'!"
                   << std::endl;
                throw std::runtime_error(ss.str());
            }

            *it_out = *it_in;
            std::vector<double> mlp_input;
            std::vector<double> mlp_output(mlp_output_size_, 0.0);
            convertNumeric(it_out->value, mlp_input);
            mlp_->compute(mlp_input.data(),mlp_output.data());
            it_out->classification = mlp_class_labels_.at(maxIndex(mlp_output));
        }
    }
    m_.unlock();

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, out);
}

void MLP::load()
{
    std::string path = readParameter<std::string>("MLP path");
    if(path == "")
        return;

    std::vector<size_t> layers;
    std::vector<double> weights;

    try {
        YAML::Node document = YAML::LoadFile(path);
        YAML::Node y_layers = document["layers"];
        for(auto it = y_layers.begin() ; it != y_layers.end() ; ++it)
            layers.push_back((*it).as<size_t>());
        weights           = document["weights"].as<std::vector<double>>();
        mlp_class_labels_ = document["classes"].as<std::vector<int>>();
    } catch (const YAML::Exception &e) {
        std::cerr << e.what() << std::endl;
        modifier_->setWarning(e.what());
        return;
    }


    m_.lock();
    mlp_.reset();
    mlp_input_size_  = 0;
    mlp_output_size_ = 0;

    if(layers.size() == 0 || weights.size() == 0) {
        modifier_->setWarning("Couldn't load layers or weights!");
        return;
    }
    if(layers.size() < 3) {
        modifier_->setWarning("MLP must have at least 3 layers!");
        return;
    }
    if(mlp_class_labels_.size() != layers.back()) {
        mlp_class_labels_.clear();
        return;
    }

    mlp_input_size_  = layers.front();
    mlp_output_size_ = layers.back();

    mlp_.reset(new mlp::MLP(layers.size(),
                            layers.data(),
                            weights.size(),
                            weights.data()));
    m_.unlock();
}
