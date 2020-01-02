#ifndef STATIC_TRANSFORM_H
#define STATIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class StaticTransform : public csapex::Node
{
public:
    StaticTransform();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Input* input_;
    Output* output_;

    std::string frame;
    std::string child_frame;

    double roll;
    double pitch;
    double yaw;
    double x;
    double y;
    double z;
};

}  // namespace csapex

#endif  // STATIC_TRANSFORM_H
