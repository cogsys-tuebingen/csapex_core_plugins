#ifndef TRANSFORM_PUBLISHER_H
#define TRANSFORM_PUBLISHER_H

/// PROJECT
#include <csapex_ros/ros_node.h>

/// SYSTEM
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <tf/transform_broadcaster.h>

namespace csapex {

class TransformPublisher : public RosNode
{
public:
    TransformPublisher();
    ~TransformPublisher();

    virtual void process();
    virtual void setup();

private:
    ConnectorIn* input_transform;
    ConnectorIn* input_time;

    tf::TransformBroadcaster* tfb_;
};

}

#endif // TRANSFORM_PUBLISHER_H
