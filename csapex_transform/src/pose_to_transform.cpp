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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>

// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <tf/tf.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

using namespace csapex::connection_types;

namespace csapex
{
class PoseToTransform : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addMultiInput<geometry_msgs::PoseStamped, geometry_msgs::PoseWithCovarianceStamped>("Pose");
        out_ = modifier.addOutput<TransformMessage>("Transform");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareText("child frame", "/pose"), child_frame_);
    }

    void process()
    {
        geometry_msgs::PoseStamped ps;

        if (msg::isMessage<GenericPointerMessage<geometry_msgs::PoseStamped>>(in_)) {
            auto pose = msg::getMessage<geometry_msgs::PoseStamped>(in_);
            ps = *pose;
        } else if (msg::isMessage<GenericPointerMessage<geometry_msgs::PoseWithCovarianceStamped>>(in_)) {
            auto pose = msg::getMessage<geometry_msgs::PoseWithCovarianceStamped>(in_);
            ps.pose = pose->pose.pose;
            ps.header = pose->header;

        } else {
            aerr << "unknown input type" << std::endl;
            return;
        }

        auto result = std::make_shared<connection_types::TransformMessage>(ps.header.frame_id, child_frame_);
        tf::Transform& trafo = result->value;

        tf::poseMsgToTF(ps.pose, trafo);

        msg::publish(out_, result);
    }

private:
    Input* in_;
    Output* out_;

    std::string child_frame_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::PoseToTransform, csapex::Node)
