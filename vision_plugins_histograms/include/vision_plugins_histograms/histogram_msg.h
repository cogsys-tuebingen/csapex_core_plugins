#ifndef HISTOGRAM_MSG_H
#define HISTOGRAM_MSG_H

/// PROJECT
#include <csapex/model/message.h>
#include "histogram.h"

namespace csapex {
namespace connection_types {

class HistogramMessage : public MessageTemplate<Histograms, HistogramMessage>
{
public:
    HistogramMessage();

};
}
}
#endif // HISTOGRAM_MSG_H
