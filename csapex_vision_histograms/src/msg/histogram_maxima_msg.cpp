/// HEADER
#include <csapex_vision_histograms/histogram_maxima_msg.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::HistogramMaximaMessage)

using namespace csapex;
using namespace connection_types;

HistogramMaximaMessage::HistogramMaximaMessage() : MessageTemplate<HistogramMaximaContainer, HistogramMaximaMessage>()
{
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const HistogramMaximaContainer& t)
{
    data << t.maxima;
    data << t.bin_range;
    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, HistogramMaximaContainer& t)
{
    data >> t.maxima;
    data >> t.bin_range;
    return data;
}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::HistogramMaximaMessage>::encode(const csapex::connection_types::HistogramMaximaMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    // TODO: implement
    return node;
}

bool convert<csapex::connection_types::HistogramMaximaMessage>::decode(const Node& node, csapex::connection_types::HistogramMaximaMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    // TODO: implement
    return true;
}
}  // namespace YAML
