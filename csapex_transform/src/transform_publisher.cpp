/// HEADER
#include "transform_publisher.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_ros/time_stamp_message.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex_ros/ros_handler.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformPublisher, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformPublisher::TransformPublisher()
    : tfb_(NULL)
{
    addParameter(param::ParameterFactory::declareText("from", "/"));
    addParameter(param::ParameterFactory::declareText("to", "/"));
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
    if(!tfb_) {
        tfb_ = new tf::TransformBroadcaster;
    }

    ros::Time time;
    if(input_time->hasMessage()) {
        TimeStampMessage::Ptr time_msg = input_time->getMessage<TimeStampMessage>();
        time = time_msg->value;
    } else {
        time = ros::Time::now();
    }

    TransformMessage::Ptr trafo_msg = input_transform->getMessage<TransformMessage>();

    tfb_->sendTransform(tf::StampedTransform(trafo_msg->value, time, readParameter<std::string>("from"), readParameter<std::string>("to")));
}


void TransformPublisher::setup()
{
    input_transform = modifier_->addInput<connection_types::TransformMessage>("T");
    input_time = modifier_->addOptionalInput<connection_types::TimeStampMessage>("time");
}
