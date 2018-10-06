/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_transform/transform_message.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

using namespace csapex::connection_types;

namespace csapex
{
class OdometryToTransform : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<nav_msgs::Odometry>("Odometry");
        out_ = modifier.addOutput<TransformMessage>("Transform");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        std::shared_ptr<nav_msgs::Odometry const> odom = msg::getMessage<nav_msgs::Odometry>(in_);

        connection_types::TransformMessage::Ptr result = std::make_shared<connection_types::TransformMessage>(odom->header.frame_id, odom->child_frame_id);
        result->stamp_micro_seconds = odom->header.stamp.toNSec() * 1e-3;
        tf::Transform& trafo = result->value;

        tf::poseMsgToTF(odom->pose.pose, trafo);

        msg::publish(out_, result);
    }

private:
    Input* in_;
    Output* out_;

    std::string child_frame_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::OdometryToTransform, csapex::Node)
