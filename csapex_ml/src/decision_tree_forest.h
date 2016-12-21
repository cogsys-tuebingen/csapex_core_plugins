#ifndef DECISION_TREE_FOREST_H
#define DECISION_TREE_FOREST_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN DecisionTreeForest : public Node
{
public:
    DecisionTreeForest();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input      *in_;
    Output     *out_;
    Slot       *reload_;
    bool        loaded_;
    std::size_t forest_size_;

    void load();

};
}

#endif // DECISION_TREE_FOREST_H
