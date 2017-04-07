/// HEADER
#include <csapex_core_plugins/timestamp_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::TimestampMessage)

using namespace csapex;
using namespace connection_types;

using namespace std::chrono;

TimestampMessage::TimestampMessage(TimestampMessage::Tp time)
    : MessageTemplate<Tp,TimestampMessage>("/", std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch()).count())
{
    value = time;
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::TimestampMessage>::encode(const csapex::connection_types::TimestampMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    node["time"] = duration_cast<microseconds>(rhs.value.time_since_epoch()).count();
    return node;
}

bool convert<csapex::connection_types::TimestampMessage>::decode(const Node& node, csapex::connection_types::TimestampMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    int64_t micro_seconds_since_epoch = node["time"].as<int64_t>();
    rhs.value = TimestampMessage::Tp(microseconds(micro_seconds_since_epoch));
    return true;
}
}

