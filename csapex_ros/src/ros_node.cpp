/// HEADER
#include <csapex_ros/ros_node.h>

/// PROJECT
#include <csapex/model/node_modifier.h>

using namespace csapex;

RosNode::RosNode()
{
}

void RosNode::getProperties(std::vector<std::string>& properties) const
{
    Node::getProperties(properties);
    properties.push_back("ROS");
}

ROSHandler& RosNode::getRosHandler() const
{
    return ROSHandler::instance();
}

void RosNode::setup(NodeModifier& node_modifier)
{
    if (isConnected()) {
        setupROS();

    } else {
        node_modifier_->setError("no connection to ROS");
        observe(getRosHandler().connected, [this]() {
            setupROS();
            yield();
        });
    }
}

bool RosNode::isConnected() const
{
    return getRosHandler().isConnected();
}

bool RosNode::canProcess() const
{
    if (Node::canProcess()) {
        if (isConnected()) {
            return true;
        } else if (!node_modifier_->isError()) {
            node_modifier_->setWarning("Cannot run, no ROS connection possible.");
        }
    }
    return false;
}

void RosNode::process()
{
    apex_assert(isConnected());

    processROS();
}

bool RosNode::canRunInSeparateProcess() const
{
    // ROS nodes cannot run in subprocess, because then ros::init would have to be
    // called for each node!
    return false;
}
