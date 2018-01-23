/// HEADER
#include "jannlab_mlp.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
#include <fstream>
#include <sstream>

CSAPEX_REGISTER_CLASS(csapex::JANNLabMLP, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

JANNLabMLP::JANNLabMLP() :
    mlp_input_size_(0),
    mlp_output_size_(0)
{
}

void JANNLabMLP::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("Features");
    out_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Labeled Features");
}

void JANNLabMLP::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::ParameterFactory::declarePath("MLP path",
                                                      csapex::param::ParameterDescription("Path to a saved MLP."),
                                                      true,
                                                      "",
                                                      "*.yaml"),
                 std::bind(&JANNLabMLP::load, this));
    addParameter(csapex::param::ParameterFactory::declarePath("normalization path",
                                                      csapex::param::ParameterDescription("Path to a normalization file."),
                                                      true,
                                                      "",
                                                      "*.norm"),
                 std::bind(&JANNLabMLP::load, this));

}

namespace {
inline unsigned int maxIndex(const std::vector<double> &nn_output)
{
    double       max = -std::numeric_limits<double>::infinity();
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


void JANNLabMLP::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> in = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<FeaturesMessage> >      out(new std::vector<FeaturesMessage>());

    std::unique_lock<std::mutex> lock(m_);
    if(mlp_input_size_ == 0 || mlp_output_size_ == 0) {
        *out = *in;
        node_modifier_->setWarning("cannot classfiy, no mlp loaded");
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
            convertNumeric(it_out->value, mlp_input);

            if(!norm_.empty()) {
                apex_assert(norm_.size() == 2);
                apex_assert(norm_[0].size() == mlp_input_size_);
                apex_assert(norm_[1].size() == mlp_input_size_);

                for(std::size_t i = 0; i < mlp_input.size(); ++i) {
                    mlp_input[i] = (mlp_input[i] - norm_[0][i]) / (norm_[1][i]);
                }
            }

            std::vector<double> mlp_output(mlp_output_size_, 0.0);

            mlp_->compute(mlp_input.data(),mlp_output.data());
            it_out->classification = mlp_class_labels_.at(maxIndex(mlp_output));
        }
    }
    lock.unlock();

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, out);
}

void JANNLabMLP::load()
{
    auto mlp_p = readParameter<std::string>("MLP path");
    auto norm_p = readParameter<std::string>("normalization path");

    if(mlp_path_ == mlp_p && norm_path_ == norm_p) {
        return;
    }

    mlp_path_ = mlp_p;
    norm_path_ = norm_p;

    if(mlp_path_.empty()) {
        return;
    }


    try {
        YAML::Node document = YAML::LoadFile(mlp_path_);
        layers_.clear();
        YAML::Node y_layers = document["layers"];
        for(auto it = y_layers.begin() ; it != y_layers.end() ; ++it)
            layers_.push_back((*it).as<size_t>());
        weights_           = document["weights"].as<std::vector<double>>();
        mlp_class_labels_ = document["classes"].as<std::vector<int>>();
    } catch (const YAML::Exception &e) {
        std::cerr << e.what() << std::endl;
        node_modifier_->setWarning(e.what());
        return;
    }

    if(!norm_path_.empty()) {
        std::string line;
        std::ifstream in(norm_path_);
        while (std::getline(in, line)) {
            norm_.emplace_back();

            std::istringstream iss(line);
            double num = 0;
            while(iss >> num)  {
                norm_.back().push_back(num);
            }
        }
    }


    std::unique_lock<std::mutex> lock(m_);
    mlp_.reset();
    mlp_input_size_  = 0;
    mlp_output_size_ = 0;

    if(layers_.size() == 0 || weights_.size() == 0) {
        node_modifier_->setWarning("Couldn't load layers or weights!");
        return;
    }
    if(layers_.size() < 3) {
        node_modifier_->setWarning("MLP must have at least 3 layers!");
        return;
    }
    if(mlp_class_labels_.size() != layers_.back()) {
        mlp_class_labels_.clear();
        return;
    }

    int connections = 0;
    for(std::size_t i = 0; i < layers_.size() - 1; ++i) {
        int l = layers_[i];
        int lnext = layers_[i+1];
        connections += (l * lnext);
    }

    if(connections != (int) weights_.size()) {
        node_modifier_->setWarning(std::string("Net has ") + std::to_string(connections) + " connections but " + std::to_string(weights_.size()) + " weights");
        return;
    }

    mlp_input_size_  = layers_.front();
    mlp_output_size_ = layers_.back();

//    mlp_.reset(new mlp::MLP(layers_.size(),
//                            layers_.data(),
//                            weights_.size(),
//                            weights_.data()));
}
