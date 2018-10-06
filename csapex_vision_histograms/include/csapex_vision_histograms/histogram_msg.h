#ifndef HISTOGRAM_MSG_H
#define HISTOGRAM_MSG_H

/// PROJECT
#include "histogram_container.h"
#include <csapex/msg/message_template.hpp>

namespace csapex
{
namespace connection_types
{
class HistogramMessage : public MessageTemplate<HistogramContainer, HistogramMessage>
{
public:
    HistogramMessage();
};

/// TRAITS
template <>
struct type<HistogramMessage>
{
    static std::string name()
    {
        return "Histogram";
    }
};

}  // namespace connection_types

SerializationBuffer& operator<<(SerializationBuffer& data, const HistogramContainer& t);
const SerializationBuffer& operator>>(const SerializationBuffer& data, HistogramContainer& t);
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct convert<csapex::connection_types::HistogramMessage>
{
    static Node encode(const csapex::connection_types::HistogramMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::HistogramMessage& rhs);
};
}  // namespace YAML
#endif  // HISTOGRAM_MSG_H
