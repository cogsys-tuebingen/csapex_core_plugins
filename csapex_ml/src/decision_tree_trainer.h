#ifndef DECISION_TREE_TRAINER_H
#define DECISION_TREE_TRAINER_H

/// PROJECT
#include <csapex_core_plugins/collection_node.h>
#include <csapex_ml/features_message.h>

namespace csapex {


class DecisionTreeTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    DecisionTreeTrainer();

    void setup();
    void setupParameters();

private:
    void processCollection(std::vector<connection_types::FeaturesMessage> &collection);

};


}

#endif // DECISION_TREE_TRAINER_H
