#ifndef NUMBER_GENERATOR_H
#define NUMBER_GENERATOR_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class NumberGenerator : public csapex::Node
{
public:
    NumberGenerator();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;

private:
    Input* input_;
    Output* output_;

    int n;
};
}  // namespace csapex

#endif  // NUMBER_GENERATOR_H
