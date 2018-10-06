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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    csapex::Output* output_;
    csapex::Input* input_;
};
}  // namespace csapex

#endif  // WATERSHED_H
