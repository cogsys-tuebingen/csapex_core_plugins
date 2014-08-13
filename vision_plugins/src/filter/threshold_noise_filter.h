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
    virtual void setupParameters();

protected:
    csapex::Output *output_;
    csapex::Input  *input_;
    csapex::Input  *threshold_;

};
}
#endif // WEIGHTED_NOISE_FILTER_H
