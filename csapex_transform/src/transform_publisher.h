#ifndef TRANSFORM_PUBLISHER_H
#define TRANSFORM_PUBLISHER_H

/// PROJECT
#include <csapex_ros/ros_node.h>

/// SYSTEM
#include <csapex/utility/suppress_warnings_start.h>
    #include <tf/transform_broadcaster.h>
#include <csapex/utility/suppress_warnings_end.h>

namespace csapex {

class TransformPublisher : public RosNode
{
public:
    TransformPublisher();
    ~TransformPublisher();

    virtual void setupROS() override;
    virtual void processROS() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

private:
    Input* input_transform;
    Input* input_time;

    tf::TransformBroadcaster* tfb_;
};

}

#endif // TRANSFORM_PUBLISHER_H
