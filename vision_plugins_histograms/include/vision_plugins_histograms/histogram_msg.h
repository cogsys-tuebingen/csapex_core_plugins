#ifndef HISTOGRAM_MSG_H
#define HISTOGRAM_MSG_H

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include "histogram_container.h"

namespace csapex {
namespace connection_types {
class HistogramMessage :
        public MessageTemplate<HistogramContainer,
                               HistogramMessage>
{
public:
    HistogramMessage();
};

/// TRAITS
template <>
struct type<HistogramMessage> {
    static std::string name() {
        return "Histogram";
    }
};

}
}
#endif // HISTOGRAM_MSG_H
