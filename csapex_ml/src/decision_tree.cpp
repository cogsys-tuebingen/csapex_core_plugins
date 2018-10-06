/// HEADER
#include "decision_tree.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/slot.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::DecisionTree, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

DecisionTree::DecisionTree() : loaded_(false)
{
}

void DecisionTree::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareFileInputPath("file", "dtree.yaml"), std::bind(&DecisionTree::loadTree, this));
}

void DecisionTree::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Unclassified feature");
    out_ = node_modifier.addOutput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Classified feature");

    reload_ = node_modifier.addSlot("Reload", std::bind(&DecisionTree::loadTree, this));
}

void DecisionTree::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<FeaturesMessage>> output(new std::vector<FeaturesMessage>);

    std::size_t n = input->size();

    if (loaded_) {
        output->resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            output->at(i) = classify(input->at(i));
        }

    } else {
        *output = *input;
        node_modifier_->setWarning("cannot classfiy, no tree loaded");
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}

connection_types::FeaturesMessage DecisionTree::classify(const FeaturesMessage& input)
{
    connection_types::FeaturesMessage result = input;

    cv::Mat feature(1, input.value.size(), CV_32FC1, result.value.data());

#if CV_MAJOR_VERSION == 2
    CvDTreeNode* node = dtree_.predict(feature);
    result.classification = node->class_idx;
#elif CV_MAJOR_VERSION == 3
    result.classification = dtree_->predict(feature);
#endif

    return result;
}

void DecisionTree::loadTree()
{
    std::string file = readParameter<std::string>("file");

#if CV_MAJOR_VERSION == 2
    dtree_.load(file.c_str());
    loaded_ = dtree_.get_root() != nullptr;
#elif CV_MAJOR_VERSION == 3
    dtree_ = cv::ml::StatModel::load<cv::ml::DTrees>(file);
#endif
}
