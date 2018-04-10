#ifndef HISTOGRAM_MAXIMA_MESSAGE_H
#define HISTOGRAM_MAXIMA_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include "histogram_maxima_container.h"

namespace csapex {
namespace connection_types {
class HistogramMaximaMessage :
        public MessageTemplate<HistogramMaximaContainer,
                               HistogramMaximaMessage>
{
public:
    HistogramMaximaMessage();
};

/// TRAITS
template <>
struct type<HistogramMaximaMessage> {
    static std::string name() {
        return "Maxima";
    }
};

}


SerializationBuffer& operator << (SerializationBuffer& data, const HistogramMaximaContainer& t);
const SerializationBuffer& operator >> (const SerializationBuffer& data, HistogramMaximaContainer& t);

}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::HistogramMaximaMessage> {
  static Node encode(const csapex::connection_types::HistogramMaximaMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::HistogramMaximaMessage& rhs);
};
}

#endif // HISTOGRAM_MAXIMA_MESSAGE_H
