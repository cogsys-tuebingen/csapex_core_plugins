#ifndef MACHINE_LEARNING_NODE_H
#define MACHINE_LEARNING_NODE_H

/// PROJECT
#include <csapex_core_plugins/collection_node.h>
#include <csapex_ml/features_message.h>

namespace csapex {

class MachineLearningNode : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    MachineLearningNode();
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    virtual bool processCollection(std::vector<connection_types::FeaturesMessage> &collection) = 0;

protected:
    bool is_classification_;
    std::string file_name_;
};
}

#endif // MACHINE_LEARNING_NODE_H
