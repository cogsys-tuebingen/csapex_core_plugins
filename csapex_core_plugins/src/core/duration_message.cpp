/// HEADER
#include <csapex_core_plugins/duration_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::DurationMessage)

using namespace csapex;
using namespace connection_types;

using namespace std::chrono;

DurationMessage::DurationMessage(std::chrono::microseconds duration, Message::Stamp stamp_micro_seconds) : MessageTemplate<std::chrono::microseconds, DurationMessage>("/", stamp_micro_seconds)
{
    value = duration;
}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::DurationMessage>::encode(const csapex::connection_types::DurationMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["duration"] = duration_cast<microseconds>(rhs.value).count();
    return node;
}

bool convert<csapex::connection_types::DurationMessage>::decode(const Node& node, csapex::connection_types::DurationMessage& rhs)
{
    convert<csapex::connection_types::Message>::decode(node, rhs);

    int64_t dur = node["duration"].as<int64_t>();
    rhs.value = microseconds(dur);
    return true;
}
}  // namespace YAML

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const std::chrono::microseconds& t)
{
    int64_t micro_seconds_since_epoch = duration_cast<microseconds>(t).count();
    data << micro_seconds_since_epoch;

    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, std::chrono::microseconds& t)
{
    int64_t micro_seconds_since_epoch;
    data >> micro_seconds_since_epoch;

    t = microseconds(micro_seconds_since_epoch);

    return data;
}
