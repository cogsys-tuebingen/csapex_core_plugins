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
}

#endif // HISTOGRAM_MAXIMA_MESSAGE_H
