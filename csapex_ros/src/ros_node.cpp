/// HEADER
#include <csapex_ros/ros_node.h>

using namespace csapex;

RosNode::RosNode()
{

}

ROSHandler& RosNode::getRosHandler()
{
    return ROSHandler::instance();
}
