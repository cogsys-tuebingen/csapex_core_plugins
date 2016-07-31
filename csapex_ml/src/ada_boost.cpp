/// HEADER
#include "ada_boost.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

CSAPEX_REGISTER_CLASS(csapex::AdaBoost, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

AdaBoost::AdaBoost() :
    loaded_(false)
{
}

void AdaBoost::setup(NodeModifier& node_modifier)
{
    in_  = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("labelled features");
}

void AdaBoost::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::ParameterFactory::declareFileOutputPath(
                                                      "boost/path",
                                                      "",
                                                      "*.yaml *.tar.gz"),
                 std::bind(&AdaBoost::load, this));

    addParameter(csapex::param::ParameterFactory::declareBool("boost/compute_labels",
                                                              false),
                 compute_labels_);
}

void AdaBoost::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr< std::vector<FeaturesMessage> > output (new std::vector<FeaturesMessage>);
    output->insert(output->end(), input->begin(), input->end());

    if(!loaded_) {
        throw std::runtime_error("Classifier ensemble is not loaded!");
    }

    std::size_t size = input->size();
    for(std::size_t i = 0 ; i < size ; ++i)
    {
        cv::Mat sample(output->at(i).value);
        if (compute_labels_)
            output->at(i).classification = boost_.predict(sample);
        else
        {
            const float response = boost_.predict(sample, cv::Mat(), cv::Range::all(), false, true);
            output->at(i).classification = std::floor(response + 0.5f);
        }
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}

void AdaBoost::load()
{
    std::string path = readParameter<std::string>("boost/path");
    if(path == "")
        return;

    boost_.load(path.c_str(), "adaboost");
    loaded_ = true;
}
