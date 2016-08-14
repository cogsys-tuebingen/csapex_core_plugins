/// HEADER
#include <csapex_vision_histograms/histogram_maxima_msg.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::HistogramMaximaMessage)

using namespace csapex;
using namespace connection_types;

HistogramMaximaMessage::HistogramMaximaMessage() :
    MessageTemplate<HistogramMaximaContainer,
                    HistogramMaximaMessage>()
{
}




/// YAML
namespace YAML {
Node convert<csapex::connection_types::HistogramMaximaMessage>::encode(const csapex::connection_types::HistogramMaximaMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    // TODO: implement
    return node;
}

bool convert<csapex::connection_types::HistogramMaximaMessage>::decode(const Node& node, csapex::connection_types::HistogramMaximaMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    // TODO: implement
    return true;
}
}
