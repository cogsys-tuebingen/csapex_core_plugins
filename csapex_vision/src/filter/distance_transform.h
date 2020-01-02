#ifndef DISTANCE_TRANSFORM_H
#define DISTANCE_TRANSFORM_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex
{
class DistanceTransform : public csapex::Node
{
public:
    DistanceTransform();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    Input* in_;
    Output* out_;
    Output* out_label_;
};

}  // namespace csapex

#endif  // DISTANCE_TRANSFORM_H
