#ifndef COPYMAKEBORDER_H
#define COPYMAKEBORDER_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class CopyMakeBorder : public Node
{
public:
    CopyMakeBorder();

    virtual void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);

private:
    Input* input_;
    Output* output_;
};
}  // namespace csapex
#endif  // COPYMAKEBORDER_H
