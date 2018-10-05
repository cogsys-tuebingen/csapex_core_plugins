/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ros/generic_ros_message.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <nav_msgs/Odometry.h>

using namespace csapex::connection_types;

namespace csapex
{
class RosTest : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<nav_msgs::Odometry>("ROS input");
    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {
    }

    void process() override
    {
        std::shared_ptr<nav_msgs::Odometry const> img_in = msg::getMessage<nav_msgs::Odometry>(in_);

        ainfo << *img_in << std::endl;
    }

private:
    Input* in_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::RosTest, csapex::Node)
