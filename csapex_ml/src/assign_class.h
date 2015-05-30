#ifndef ASSIGNCLASS_H
#define ASSIGNCLASS_H

#include <csapex/model/node.h>

namespace csapex {
class AssignClass : public Node
{
public:
    AssignClass();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    csapex::Input  *in_features_;
    csapex::Input  *in_labels_;
    csapex::Output *out_;
};
}

#endif // ASSIGNCLASS_H
