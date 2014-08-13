#ifndef DECISION_TREE_TRAINER_H
#define DECISION_TREE_TRAINER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_ml/features_message.h>

namespace csapex {


class DecisionTreeTrainer : public csapex::Node
{
public:
    DecisionTreeTrainer();

    void setupParameters();
    void setup();
    void process();

private:
    void train();

private:
    Input* in_;
    Output* out_;

    std::vector<connection_types::FeaturesMessage> features_;
};


}

#endif // DECISION_TREE_TRAINER_H
