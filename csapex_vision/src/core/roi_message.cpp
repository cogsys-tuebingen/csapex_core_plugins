/// HEADER
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>
#include <csapex_vision/yaml_io.hpp>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::RoiMessage)

using namespace csapex;
using namespace connection_types;


RoiMessage::RoiMessage()
    : MessageTemplate<Roi, RoiMessage> ("/")
{}


/// YAML
namespace YAML {
Node convert<csapex::connection_types::RoiMessage>::encode(const csapex::connection_types::RoiMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    node["value"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::RoiMessage>::decode(const Node& node, csapex::connection_types::RoiMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);

    rhs.value = node["value"].as<Roi>();
    return true;
}
}
