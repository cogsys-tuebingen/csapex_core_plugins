/// HEADER
#include <csapex_math/msg/linear_matrix_message.h>

/// COMPONENT
#include <csapex_math/serialization/yaml_io.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::LinearMatrixMessage)

using namespace csapex;
using namespace csapex::connection_types;

LinearMatrixMessage::LinearMatrixMessage(math::linear::Matrix v, const std::string& frame_id, Message::Stamp stamp) : Parent(frame_id, stamp)
{
    value = v;
}

/// YAML
namespace YAML
{
CSAPEX_EXPORT_PLUGIN Node convert<csapex::connection_types::LinearMatrixMessage>::encode(const csapex::connection_types::LinearMatrixMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    node["value"] = rhs.value;
    return node;
}

CSAPEX_EXPORT_PLUGIN bool convert<csapex::connection_types::LinearMatrixMessage>::decode(const Node& node, csapex::connection_types::LinearMatrixMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);

    rhs.value = node["value"].as<math::linear::Vector>();
    return true;
}
}  // namespace YAML
