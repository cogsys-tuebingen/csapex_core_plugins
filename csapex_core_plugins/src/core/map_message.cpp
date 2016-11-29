///HEADER
#include <csapex_core_plugins/map_message.h>
/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::MapMessage)

using namespace csapex;
using namespace connection_types;

using namespace std::chrono;

MapMessage::MapMessage(std::size_t size, const std::string &frame_id, Stamp stamp)
    : MessageTemplate<std::vector<KeyValueMessage>,MapMessage>(frame_id, stamp)
{
    value = std::vector<KeyValueMessage>(size);
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::MapMessage>::encode(const csapex::connection_types::MapMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["values"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::MapMessage>::decode(const Node& node, csapex::connection_types::MapMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    rhs.value = node["values"].as<std::vector<KeyValueMessage>>();

    return true;
}
}
