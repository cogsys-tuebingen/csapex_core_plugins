/// HEADER
#include <csapex_ros/ros_node.h>

/// PROJECT
#include <csapex/model/node_modifier.h>

using namespace csapex;

RosNode::RosNode()
    : ros_init_(false)
{

}

ROSHandler& RosNode::getRosHandler() const
{
    return ROSHandler::instance();
}

void RosNode::setup(NodeModifier& node_modifier)
{
}

void RosNode::ensureROSisSetUp()
{
    if(!ros_init_) {
        if(isConnected()) {
            setupROS();
            ros_init_ = true;
        }
    }
}

bool RosNode::canTick()
{
    ROSHandler& ros = getRosHandler();
    return ros.isConnected();
}

void RosNode::tick()
{
    ROSHandler& ros = getRosHandler();
    if(isConnected()) {
        ensureROSisSetUp();
        tickROS();

    } else if(ros_init_) {
        ros_init_ = false;
        node_modifier_->setWarning("[tick] no ROS connection");
    }
}

bool RosNode::tickROS()
{
    return false;
}

bool RosNode::isConnected() const
{
    return getRosHandler().isConnected();
}

void RosNode::process()
{
    if(isConnected()) {
        ensureROSisSetUp();
        processROS();

    } else if(ros_init_) {
        ros_init_ = false;
        node_modifier_->setWarning("[process] no ROS connection");
    }
}
