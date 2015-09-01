#ifndef ROS_NODE_H
#define ROS_NODE_H

/// COMPONENT
#include <csapex_ros/ros_handler.h>

/// PROJECT
#include <csapex/model/tickable_node.h>

/// SYSTEM
#include <ros/ros.h>

namespace csapex
{

class RosNode : public TickableNode
{
protected:
    RosNode();

    virtual void process() override;
    virtual void setup(NodeModifier& node_modifier) override;

    virtual void setupROS() = 0;
    virtual void processROS() = 0;

    virtual bool canTick() override;
    virtual void tick() override;
    virtual void tickROS();

    ROSHandler& getRosHandler();

private:
    bool ros_init_;
};

}

#endif // ROS_NODE_H
