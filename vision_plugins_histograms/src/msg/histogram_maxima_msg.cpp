/// HEADER
#include <vision_plugins_histograms/histogram_maxima_msg.h>

using namespace csapex;
using namespace connection_types;

HistogramMaximaMessage::HistogramMaximaMessage() :
    MessageTemplate<HistogramMaximaContainer,
                    HistogramMaximaMessage>("Maxima")
{
}
