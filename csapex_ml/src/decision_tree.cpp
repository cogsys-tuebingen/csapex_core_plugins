/// HEADER
#include "decision_tree.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/signal/slot.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::DecisionTree, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


DecisionTree::DecisionTree()
    : loaded_(false)
{
}

void DecisionTree::setupParameters()
{
    addParameter(param::ParameterFactory::declareFileInputPath("file", "dtree.yaml"), boost::bind(&DecisionTree::loadTree, this));
}

void DecisionTree::setup()
{
    in_  = modifier_->addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Unclassified feature");
    out_ = modifier_->addOutput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Classified feature");

    reload_ = modifier_->addSlot("Reload", boost::bind(&DecisionTree::loadTree, this));
}

void DecisionTree::process()
{
    boost::shared_ptr<std::vector<FeaturesMessage> const> input = in_->getMessage<GenericVectorMessage, FeaturesMessage>();
    boost::shared_ptr< std::vector<FeaturesMessage> > output (new std::vector<FeaturesMessage>);

    std::size_t n = input->size();

    if(loaded_) {
        output->resize(n);
        for(std::size_t i = 0; i < n; ++i) {
            output->at(i) = classify(input->at(i));
        }

    } else {
        *output = *input;
        setError(true, "cannot classfiy, no tree loaded", EL_WARNING);
    }

    out_->publish<GenericVectorMessage, FeaturesMessage>(output);
}

connection_types::FeaturesMessage DecisionTree::classify(const FeaturesMessage &input)
{
    connection_types::FeaturesMessage result = input;

    cv::Mat feature(1, input.value.size(), CV_32FC1, result.value.data());
    CvDTreeNode* node = dtree_.predict(feature);

    result.classification = node->class_idx;
    return result;
}

void DecisionTree::loadTree()
{
    std::string file = readParameter<std::string>("file");
    dtree_.load(file.c_str());

    loaded_ = dtree_.get_root() != nullptr;
}
