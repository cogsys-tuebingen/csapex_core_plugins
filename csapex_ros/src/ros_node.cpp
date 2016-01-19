/// HEADER
#include <csapex_ros/ros_node.h>

/// PROJECT
#include <csapex/model/node_modifier.h>

using namespace csapex;

RosNode::RosNode()
    : ros_init_(false)
{

}

ROSHandler& RosNode::getRosHandler()
{
    return ROSHandler::instance();
}

void RosNode::setup(NodeModifier& node_modifier)
{
}

bool RosNode::canTick()
{
    ROSHandler& ros = getRosHandler();
    return ros.isConnected();
}

void RosNode::tick()
{
    ROSHandler& ros = getRosHandler();
    if(ros.isConnected()) {
        if(!ros_init_) {
            setupROS();
            ros_init_ = true;
        }

        tickROS();

    } else {
        ros_init_ = false;
        node_modifier_->setWarning("[tick] no ROS connection");
    }
}

void RosNode::tickROS()
{

}

void RosNode::process()
{
    ROSHandler& ros = getRosHandler();
    if(ros.isConnected()) {
        if(!ros_init_) {
            setupROS();
            ros_init_ = true;
        }

        processROS();

    } else {
        ros_init_ = false;
        node_modifier_->setWarning("[process] no ROS connection");
    }
}
