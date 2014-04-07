#ifndef NOISE_FILTER_PLUGIN_H
#define NOISE_FILTER_PLUGIN_H

/// COMPONENT
#include <csapex/model/node.h>
#include <utils_cv/noise_filter.h>

namespace vision_plugins {
class NoiseFilter : public csapex::Node
{
public:
    NoiseFilter();

    virtual void process();
    virtual void setup();

protected:
    enum Type {RANDOM, TEMPORAL};
    csapex::ConnectorOut*       probs_out_;
    csapex::ConnectorOut*       output_;
    csapex::ConnectorIn*        input_;
    utils_cv::NoiseFilter::Ptr  noise_filter_;

    void update();
};
}
#endif // NOISE_FILTER_PLUGIN_H
