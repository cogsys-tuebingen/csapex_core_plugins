/// HEADER
#include "transform_publisher.h"

/// COMPONENT
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_ros/ros_handler.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformPublisher, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformPublisher::TransformPublisher() : tfb_(nullptr)
{
}

void TransformPublisher::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareText("from", "/"));
    parameters.addParameter(csapex::param::factory::declareText("to", "/"));
}

TransformPublisher::~TransformPublisher()
{
    delete tfb_;
}

void TransformPublisher::setupROS()
{
}

void TransformPublisher::processROS()
{
    if (!tfb_) {
        tfb_ = new tf::TransformBroadcaster;
    }

    TransformMessage::ConstPtr trafo_msg = msg::getMessage<TransformMessage>(input_transform);

    std::string from = readParameter<std::string>("from");
    if (from == "/" || from.empty()) {
        from = trafo_msg->frame_id;
    }

    std::string to = readParameter<std::string>("to");
    if (to == "/" || to.empty()) {
        to = trafo_msg->child_frame;
    }

    ros::Time time;
    if (msg::hasMessage(input_time)) {
        TimestampMessage::ConstPtr time_msg = msg::getMessage<TimestampMessage>(input_time);
        auto nano = std::chrono::duration_cast<std::chrono::nanoseconds>(time_msg->value.time_since_epoch());
        time.fromNSec(nano.count());
    } else {
        time = ros::Time::now();
    }

    tfb_->sendTransform(tf::StampedTransform(trafo_msg->value, time, from, to));
}

void TransformPublisher::setup(NodeModifier& node_modifier)
{
    input_transform = node_modifier.addInput<connection_types::TransformMessage>("T");
    input_time = node_modifier.addOptionalInput<connection_types::TimestampMessage>("time");
}
