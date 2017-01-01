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

    virtual bool canProcess() const override;
    virtual void process() override;
    virtual void setup(NodeModifier& node_modifier) override;

    virtual void setupROS() = 0;
    virtual void processROS() = 0;

    virtual void getProperties(std::vector<std::string>& properties) const override;


    ROSHandler& getRosHandler() const;

protected:
    bool isConnected() const;

private:
    slim_signal::ScopedConnection connection_;
};

}

#endif // ROS_NODE_H
