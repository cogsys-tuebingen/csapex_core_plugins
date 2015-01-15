/// HEADER
#include "extract_timestamp.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ExtractTimeStamp, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ExtractTimeStamp::ExtractTimeStamp()
{
}

void ExtractTimeStamp::setup()
{
    input_ = modifier_->addInput<AnyMessage>("Message");

    output_ = modifier_->addOutput<TimeStampMessage>("Time");
}

void ExtractTimeStamp::process()
{
    Message::ConstPtr msg = input_->getMessage<Message>();

    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    time->value = time->value.fromNSec(msg->stamp_micro_seconds * 1e3);
    time->stamp_micro_seconds = msg->stamp_micro_seconds;
    output_->publish(time);
}

