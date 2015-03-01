#ifndef FEATURE_TO_HISTOGRAM_H
#define FEATURE_TO_HISTOGRAM_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class FeatureToHistogram : public csapex::Node
{
public:
    FeatureToHistogram();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

protected:
    csapex::Input  *in_;
    csapex::Input  *in_vector_;
    csapex::Output *out_;

};

}
#endif // FEATURE_TO_HISTOGRAM_H
