#ifndef ASSIGNCLASS_H
#define ASSIGNCLASS_H

#include <csapex_core_plugins/vector_process_node.h>
#include <csapex_ml/features_message.h>

namespace csapex {
class AssignClass : public VectorProcessNode<connection_types::FeaturesMessage>
{
public:
    AssignClass();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

protected:
    void processCollection(std::vector<connection_types::FeaturesMessage *> &collection) override;

private:
    csapex::Input  *in_labels_;
    int label_;
};
}

#endif // ASSIGNCLASS_H
