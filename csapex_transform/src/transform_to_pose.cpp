/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>

#include <csapex/utility/suppress_warnings_start.h>
    #include <tf/tf.h>
#include <csapex/utility/suppress_warnings_end.h>

using namespace csapex::connection_types;


namespace csapex
{

class TransformToPose : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<TransformMessage>("Transform");
        out_ = modifier.addOutput<geometry_msgs::PoseStamped>("Pose");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        TransformMessage::ConstPtr trafo = msg::getMessage<TransformMessage>(in_);
        tf::Transform fixed_to_container = trafo->value;

        auto result = std::make_shared<geometry_msgs::PoseStamped>();
        geometry_msgs::PoseStamped& ps = *result;

        tf::poseTFToMsg(fixed_to_container, ps.pose);
        ps.header.frame_id = trafo->frame_id;
        ps.header.stamp.fromNSec(trafo->stamp_micro_seconds * 1e3);

        msg::publish(out_, result);
    }
private:
    Input* in_;
    Output* out_;
};

}

CSAPEX_REGISTER_CLASS(csapex::TransformToPose, csapex::Node)

