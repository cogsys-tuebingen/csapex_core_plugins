#ifndef HOG_EXTRACTOR_H
#define HOG_EXTRACTOR_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/param/range_parameter.h>

namespace vision_plugins {
class HOGExtractor : public csapex::Node
{
public:
    HOGExtractor();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    csapex::Input                  *in_img_;
    csapex::Input                  *in_rois_;
    csapex::Output                 *out_;

    param::RangeParameter::Ptr       overlap_;
    void updateOverlap();

};
}
#endif // HOG_EXTRACTOR_H
