/// HEADER
#include "ada_boost.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

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
    reload_ = node_modifier.addSlot("Reload", std::bind(&AdaBoost::reload, this));
}

void AdaBoost::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::ParameterFactory::declareFileInputPath(
                                                      "/adaboost/path",
                                                      "",
                                                      "*.yaml *.tar.gz"),
                 path_);

    addParameter(csapex::param::ParameterFactory::declareBool("/adaboost/compute_labels",
                                                              false),
                 compute_labels_);
}

void AdaBoost::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr< std::vector<FeaturesMessage> > output (new std::vector<FeaturesMessage>);
    output->insert(output->end(), input->begin(), input->end());

    if(!loaded_) {
        if(path_ != "") {

#if CV_MAJOR_VERSION == 2
            boost_.load(path_.c_str(), "adaboost");
#elif CV_MAJOR_VERSION == 3
            boost_->load<cv::ml::Boost>(path_, "adaboost");
#endif
            loaded_ = true;
        } else {
            throw std::runtime_error("Classifier ensemble is not loaded!");
        }
    }

    std::size_t size = input->size();
    for(std::size_t i = 0 ; i < size ; ++i)
    {
        cv::Mat sample(output->at(i).value);
        if (compute_labels_)
        {
#if CV_MAJOR_VERSION == 2
            output->at(i).classification = boost_.predict(sample);
#elif CV_MAJOR_VERSION == 3
            output->at(i).classification = boost_->predict(sample);
#endif
        }
        else
        {
#if CV_MAJOR_VERSION == 2
            const float response = boost_.predict(sample, cv::Mat(), cv::Range::all(), false, false);
#elif CV_MAJOR_VERSION == 3
            const float response = boost_->predict(sample, cv::noArray(), cv::ml::StatModel::RAW_OUTPUT);
#endif
            output->at(i).classification = std::floor(response + 0.5f);
        }
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}


void AdaBoost::reload()
{
    loaded_ = false;
}

void AdaBoost::updateMethod()
{
    compute_labels_ = readParameter<bool>("/adaboost/compute_labels");
    if(compute_labels_) {
        node_modifier_->setNoError();
    } else {
        node_modifier_->setWarning("This feature is not implemented fully yet!");
    }
}

