#ifndef HISTOGRAM_MAXIMA_MESSAGE_H
#define HISTOGRAM_MAXIMA_MESSAGE_H

/// PROJECT
#include "histogram_maxima_container.h"
#include <csapex/msg/message_template.hpp>

namespace csapex
{
namespace connection_types
{
class HistogramMaximaMessage : public MessageTemplate<HistogramMaximaContainer, HistogramMaximaMessage>
{
public:
    HistogramMaximaMessage();
};

/// TRAITS
template <>
struct type<HistogramMaximaMessage>
{
    static std::string name()
    {
        return "Maxima";
    }
};

}  // namespace connection_types

SerializationBuffer& operator<<(SerializationBuffer& data, const HistogramMaximaContainer& t);
const SerializationBuffer& operator>>(const SerializationBuffer& data, HistogramMaximaContainer& t);

}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct convert<csapex::connection_types::HistogramMaximaMessage>
{
    static Node encode(const csapex::connection_types::HistogramMaximaMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::HistogramMaximaMessage& rhs);
};
}  // namespace YAML

#endif  // HISTOGRAM_MAXIMA_MESSAGE_H
