#ifndef WATERSHED_H
#define WATERSHED_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class WaterShed : public csapex::Node
{
public:
    WaterShed();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    csapex::Output* output_;
    csapex::Input* input_;
};
}  // namespace csapex

#endif  // WATERSHED_H
