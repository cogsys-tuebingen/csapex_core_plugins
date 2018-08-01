/// HEADER
#include "waldboost.h"

/// PROJECT
#include <csapex_ml/features_message.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::WaldBoost, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

WaldBoost::WaldBoost() :
    loaded_(false)
{

}

void WaldBoost::setup(NodeModifier &node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("labelled features");

    reload_ = node_modifier.addSlot("Reload", std::bind(&WaldBoost::reload, this));
}

void WaldBoost::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::factory::declarePath("/waldboost/path",
                                                                 csapex::param::ParameterDescription("Path to a saved svm."),
                                                                 true,
                                                                 "",
                                                                 "*.yaml *.tar.gz"),
                            path_);
}

void WaldBoost::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr< std::vector<FeaturesMessage> > output (new std::vector<FeaturesMessage>);
    output->insert(output->end(), input->begin(), input->end());

    if(!loaded_) {
        if(path_ != "") {
            wb_.load(path_);
            loaded_ = true;
        } else {
            throw std::runtime_error("Classifier ensemble is not loaded!");
        }
    }

    std::size_t size = input->size();
    for(std::size_t i = 0 ; i < size ; ++i)
    {
        cv::Mat sample(output->at(i).value);
        output->at(i).classification = wb_.predict(sample, output->at(i).confidence);

    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}

void WaldBoost::reload()
{
    loaded_ = false;
}
