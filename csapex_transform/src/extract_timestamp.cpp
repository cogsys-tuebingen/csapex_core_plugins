/// HEADER
#include "extract_timestamp.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/timestamp_message.h>

CSAPEX_REGISTER_CLASS(csapex::ExtractTimeStamp, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ExtractTimeStamp::ExtractTimeStamp()
{
}

void ExtractTimeStamp::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<AnyMessage>("Message");

    output_ = node_modifier.addOutput<TimestampMessage>("Time");
}

void ExtractTimeStamp::process()
{
    Message::ConstPtr msg = msg::getMessage<Message>(input_);

    connection_types::TimestampMessage::Ptr time(new connection_types::TimestampMessage);
    auto mis = std::chrono::microseconds(msg->stamp_micro_seconds);
    time->value = connection_types::TimestampMessage::Tp(mis);
    time->stamp_micro_seconds = msg->stamp_micro_seconds;
    msg::publish(output_, time);
}
