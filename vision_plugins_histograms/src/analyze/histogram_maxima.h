#ifndef HISTOGRAM_MAXIMA_H
#define HISTOGRAM_MAXIMA_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class HistogramMaxima : public csapex::Node
{
public:
    HistogramMaxima();

    virtual void process();
    virtual void setup();

protected:
    csapex::ConnectorOut* maxima_;
    csapex::ConnectorIn*  histograms_;

};
}
#endif // HISTOGRAM_MAXIMA_H
