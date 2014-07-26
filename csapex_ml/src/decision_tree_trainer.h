#ifndef DECISION_TREE_TRAINER_H
#define DECISION_TREE_TRAINER_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class DecisionTreeTrainer : public csapex::Node
{
public:
    DecisionTreeTrainer();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_;
    Output* out_;
};


}

#endif // DECISION_TREE_TRAINER_H
