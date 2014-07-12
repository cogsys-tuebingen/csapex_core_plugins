/// HEADER
#include <vision_plugins_histograms/histogram_msg.h>

using namespace csapex;
using namespace connection_types;

HistogramMessage::HistogramMessage() :
    MessageTemplate<HistogramContainer,
                    HistogramMessage>()
{
}

