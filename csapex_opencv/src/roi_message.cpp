/// HEADER
#include <csapex_opencv/roi_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>
#include <csapex_opencv/yaml_io.hpp>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::RoiMessage)

using namespace csapex;
using namespace connection_types;

RoiMessage::RoiMessage() : MessageTemplate<Roi, RoiMessage>("/")
{
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const Roi& roi)
{
    data << roi.x();
    data << roi.y();
    data << roi.w();
    data << roi.h();

    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, Roi& roi)
{
    int x, y, w, h;

    data >> x;
    data >> y;
    data >> w;
    data >> h;

    roi = Roi(x, y, w, h);

    return data;
}

/// YAML
namespace YAML
{
CSAPEX_EXPORT_PLUGIN Node convert<csapex::connection_types::RoiMessage>::encode(const csapex::connection_types::RoiMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    node["value"] = rhs.value;
    return node;
}

CSAPEX_EXPORT_PLUGIN bool convert<csapex::connection_types::RoiMessage>::decode(const Node& node, csapex::connection_types::RoiMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);

    rhs.value = node["value"].as<Roi>();
    return true;
}
}  // namespace YAML
