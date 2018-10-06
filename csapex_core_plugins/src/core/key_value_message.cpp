/// HEADER
#include <csapex_core_plugins/key_value_message.h>

/// PROJECT
#include <csapex/msg/any_message.h>
#include <csapex/msg/token_traits.h>
#include <csapex/serialization/yaml.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::KeyValueMessage)

using namespace csapex;
using namespace connection_types;

using namespace std::chrono;

KeyValueMessage::KeyValueMessage(const std::string& frame_id, Stamp stamp) : MessageTemplate<std::pair<std::string, TokenData::ConstPtr>, KeyValueMessage>(frame_id, stamp)
{
}

KeyValueMessage::KeyValueMessage(std::string name, TokenData::ConstPtr msg, const std::string& frame_id, Stamp stamp)
  : MessageTemplate<std::pair<std::string, TokenData::ConstPtr>, KeyValueMessage>(frame_id, stamp)
{
    value = std::pair<std::string, TokenData::ConstPtr>(name, msg);
}

// SerializationBuffer& operator << (SerializationBuffer& data, const
// KeyValueMessage& t)
//{
//    data << t.value;
//    return data;
//}
// const SerializationBuffer& operator >> (const SerializationBuffer& data,
// KeyValueMessage& t)
//{
//    data >> t.value;
//    return data;
//}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::KeyValueMessage>::encode(const csapex::connection_types::KeyValueMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["name"] = rhs.value.first;
    node["value"] = rhs.value.second;
    return node;
}

bool convert<csapex::connection_types::KeyValueMessage>::decode(const Node& node, csapex::connection_types::KeyValueMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    std::string name = node["name"].as<std::string>();

    TokenData::ConstPtr data_ptr = node["value"].as<TokenDataConstPtr>();
    rhs.value = std::make_pair(name, data_ptr);
    return true;
}
}  // namespace YAML
