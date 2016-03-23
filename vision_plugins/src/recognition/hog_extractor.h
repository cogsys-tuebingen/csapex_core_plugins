#ifndef HOG_EXTRACTOR_H
#define HOG_EXTRACTOR_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/param/range_parameter.h>
#include "hog.h"

/// EXTRACT HOG FEATURE

namespace vision_plugins {
class HOGExtractor : public csapex::Node
{
public:
    HOGExtractor();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    enum ClassificationType {BACKGROUND = 0, HUMAN = 1, HUMAN_PART = 2, UNKNOWN = 3};

    HOGDescriptor   hog_;
    csapex::Input  *in_img_;
    csapex::Input  *in_rois_;
    csapex::Output *out_;

    csapex::param::RangeParameter::Ptr overlap_;
    void updateOverlap();



};
}
#endif // HOG_EXTRACTOR_H
