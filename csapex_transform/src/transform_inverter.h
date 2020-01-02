#ifndef TRANSFORM_INVERTER_H
#define TRANSFORM_INVERTER_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class TransformInverter : public csapex::Node
{
public:
    TransformInverter();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

#endif  // TRANSFORM_INVERTER_H
