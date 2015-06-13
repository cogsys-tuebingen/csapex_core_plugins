#ifndef SETCOLOR_H
#define SETCOLOR_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class SetColor : public Node
{
public:
    SetColor();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Input  *input_;
    Input  *input_mask_;
    Output *output_;


};
}
#endif // SETCOLOR_H
