/// HEADER
#include "extract_timestamp.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>

CSAPEX_REGISTER_CLASS(csapex::ExtractTimeStamp, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ExtractTimeStamp::ExtractTimeStamp()
{
}

void ExtractTimeStamp::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<AnyMessage>("Message");

    output_ = node_modifier.addOutput<TimeStampMessage>("Time");
}

void ExtractTimeStamp::process()
{
    Message::ConstPtr msg = msg::getMessage<Message>(input_);

    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    time->value = time->value.fromNSec(msg->stamp_micro_seconds * 1e3);
    time->stamp_micro_seconds = msg->stamp_micro_seconds;
    msg::publish(output_, time);
}

