#ifndef ROS_NODE_H
#define ROS_NODE_H

/// COMPONENT
#include <csapex_ros/ros_handler.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <ros/ros.h>

namespace csapex
{

class RosNode : public Node
{
protected:
    RosNode();

    ROSHandler& getRosHandler();
};

}

#endif // ROS_NODE_H
