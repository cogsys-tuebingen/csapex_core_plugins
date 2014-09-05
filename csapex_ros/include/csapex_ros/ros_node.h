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

    void process();

    virtual void setupROS() = 0;
    virtual void processROS() = 0;

    virtual void tick();
    virtual void tickROS();

    ROSHandler& getRosHandler();

private:
    bool ros_init_;
};

}

#endif // ROS_NODE_H
