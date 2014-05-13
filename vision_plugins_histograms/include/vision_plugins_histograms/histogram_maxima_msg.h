#ifndef HISTOGRAM_MAXIMA_MESSAGE_H
#define HISTOGRAM_MAXIMA_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>
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
}
}

#endif // HISTOGRAM_MAXIMA_MESSAGE_H
