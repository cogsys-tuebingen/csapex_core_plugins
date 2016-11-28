/// HEADER
#include <csapex_core_plugins/key_value_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::KeyValueMessage)

using namespace csapex;
using namespace connection_types;

using namespace std::chrono;

KeyValueMessage::KeyValueMessage(std::string name, TokenData::Ptr msg)
    : MessageTemplate<std::pair<std::string, TokenData::Ptr>,KeyValueMessage>("/", stamp)
{
    value = std::pair<std::string, TokenData::Ptr>(name,msg);
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::KeyValueMessage>::encode(const csapex::connection_types::KeyValueMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["msg_name"] = rhs.value.first;
    node[rhs.value.first] = rhs.value.second;
    return node;
}

bool convert<csapex::connection_types::KeyValueMessage>::decode(const Node& node, csapex::connection_types::KeyValueMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    std::string name = node["msg_name"].as<std::string>();
    TokenData::Ptr  data;

    rhs.value = std::make_pair<std::string, TokenData::Ptr>(name, data);
    return true;
}
}

