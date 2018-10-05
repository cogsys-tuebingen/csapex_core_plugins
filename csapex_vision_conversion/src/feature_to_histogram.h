#ifndef FEATURE_TO_HISTOGRAM_H
#define FEATURE_TO_HISTOGRAM_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class FeatureToHistogram : public csapex::Node
{
public:
    FeatureToHistogram();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

protected:
    csapex::Input* in_;
    csapex::Input* in_vector_;
    csapex::Output* out_;
};

}  // namespace csapex
#endif  // FEATURE_TO_HISTOGRAM_H
