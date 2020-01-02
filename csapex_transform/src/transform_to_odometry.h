#ifndef TRANSFORM_TO_ODOMETRY_H
#define TRANSFORM_TO_ODOMETRY_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex
{
class TransformToOdometry : public csapex::Node
{
public:
    TransformToOdometry();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    Input* in_;
    Output* out_;
};

}  // namespace csapex

#endif  // TRANSFORM_TO_ODOMETRY_H
