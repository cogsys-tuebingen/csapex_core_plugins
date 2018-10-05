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

    bool canProcess() const override;
    void process() override;
    void setup(NodeModifier& node_modifier) override;

    virtual void setupROS() = 0;
    virtual void processROS() = 0;

    void getProperties(std::vector<std::string>& properties) const override;

    bool canRunInSeparateProcess() const override;

    ROSHandler& getRosHandler() const;

protected:
    bool isConnected() const;
};

}  // namespace csapex

#endif  // ROS_NODE_H
