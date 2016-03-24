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
    enum AdaptionType {SCALE, TRY_GROW, GROW_STRICT};

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
    int             adaption_type_;
    double          ratio_hog_;

    void getData(const cv::Mat &src, const cv::Rect &roi, cv::Mat &dst);
};
}
#endif // HOG_EXTRACTOR_H
