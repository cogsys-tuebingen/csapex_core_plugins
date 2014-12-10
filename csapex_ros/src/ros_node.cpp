/// HEADER
#include <csapex_ros/ros_node.h>

using namespace csapex;

RosNode::RosNode()
    : ros_init_(false)
{

}

ROSHandler& RosNode::getRosHandler()
{
    return ROSHandler::instance();
}

void RosNode::setup()
{
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
        setError(true, "[tick] no ROS connection", EL_WARNING);
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
        setError(true, "[process] no ROS connection", EL_WARNING);
    }
}
