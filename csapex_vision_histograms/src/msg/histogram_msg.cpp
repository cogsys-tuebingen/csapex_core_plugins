/// HEADER
#include <csapex_vision_histograms/histogram_msg.h>

/// PROJECT
#include <csapex/utility/register_msg.h>
#include <csapex_opencv/binary_io.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::HistogramMessage)

using namespace csapex;
using namespace connection_types;

HistogramMessage::HistogramMessage() :
    MessageTemplate<HistogramContainer,
                    HistogramMessage>()
{
}

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const HistogramContainer& t)
{
    data << t.ranges;
    data << t.histograms;
    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, HistogramContainer& t)
{
    data >> t.ranges;
    data >> t.histograms;
    return data;
}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::HistogramMessage>::encode(const csapex::connection_types::HistogramMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    // TODO: implement
    return node;
}

bool convert<csapex::connection_types::HistogramMessage>::decode(const Node& node, csapex::connection_types::HistogramMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    // TODO: implement
    return true;
}
}
