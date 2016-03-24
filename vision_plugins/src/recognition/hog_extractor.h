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
    HOGDescriptor   hog_;
    csapex::Input  *in_img_;
    csapex::Input  *in_rois_;
    csapex::Output *out_;
    bool            mirror_;
    int             cells_x_;
    int             cells_y_;
    int             cell_size_;
    int             block_size_;
    int             block_stride_;

};
}
#endif // HOG_EXTRACTOR_H
