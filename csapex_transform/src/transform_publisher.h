#ifndef TRANSFORM_PUBLISHER_H
#define TRANSFORM_PUBLISHER_H

/// PROJECT
#include <csapex_ros/ros_node.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <tf/transform_broadcaster.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

namespace csapex
{
class TransformPublisher : public RosNode
{
public:
    TransformPublisher();
    ~TransformPublisher();

    void setupROS() override;
    void processROS() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Input* input_transform;
    Input* input_time;

    tf::TransformBroadcaster* tfb_;
};

}  // namespace csapex

#endif  // TRANSFORM_PUBLISHER_H
