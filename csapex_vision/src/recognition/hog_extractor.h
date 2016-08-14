#ifndef HOG_EXTRACTOR_H
#define HOG_EXTRACTOR_H

/// PROJECT
#include <csapex/model/node.h>
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
    enum ClassificationType {BACKGROUND = 0, HUMAN = 1, HUMAN_PART = 2, UNKNOWN = 3};

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

    bool getData(const cv::Mat &src, const cv::Rect &roi, cv::Mat &dst);
};
}
#endif // HOG_EXTRACTOR_H
