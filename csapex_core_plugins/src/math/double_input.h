#ifndef DOUBLE_INPUT_H
#define DOUBLE_INPUT_H

/// PROJECT
#include <csapex/model/tickable_node.h>

namespace csapex {

template <typename T>
class NumberInput : public TickableNode
{
public:
    NumberInput();

    void tick() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    void process() override;

private:
    Output* out_;
};

}

#endif // DOUBLE_INPUT_H
