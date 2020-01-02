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

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Input* input_;
    Input* input_mask_;
    Output* output_;
};
}  // namespace csapex
#endif  // SETCOLOR_H
