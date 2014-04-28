#ifndef WEIGHTED_NOISE_FILTER_H
#define WEIGHTED_NOISE_FILTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class ThresholdNoiseFilter : public csapex::Node
{
public:
    ThresholdNoiseFilter();

    virtual void process();
    virtual void setup();

protected:
    csapex::ConnectorOut *output_;
    csapex::ConnectorIn  *input_;
    csapex::ConnectorIn  *threshold_;

};
}
#endif // WEIGHTED_NOISE_FILTER_H
