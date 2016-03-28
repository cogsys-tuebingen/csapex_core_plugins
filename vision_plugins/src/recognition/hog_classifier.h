#ifndef HOGCLASSIFIER_H
#define HOGCLASSIFIER_H

/// PROJECT
#include <csapex/model/node.h>
#include "hog.h"

/// https://github.com/DaHoC/trainHOG/wiki/trainHOG-Tutorial
/// TO CLASSIFY ROI DIRECTLY

namespace vision_plugins {
class HOGClassifier : public csapex::Node
{
public:
    HOGClassifier();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    enum SVMType  {DEFAULT = 0, DAIMLER, CUSTOM};
    enum AdaptionType {SCALE, TRY_GROW, GROW_STRICT};
    enum ClassificationType {BACKGROUND = 0, HUMAN = 1, HUMAN_PART = 2, UNKNOWN = 3};

    csapex::Input  *in_img_;
    csapex::Input  *in_rois_;
    csapex::Output *out_rois_;

    HOGDescriptor   hog_;
    bool            mirror_;
    int             cells_x_;
    int             cells_y_;
    int             cell_size_;
    int             block_size_;
    int             block_stride_;
    int             adaption_type_;
    double          ratio_hog_;

    std::vector<float> svm_;
    int                svm_type_;
    double             svm_thresh_;

    void getData(const cv::Mat &src, const cv::Rect &roi, cv::Mat &dst);
    void load();
};
}
#endif // HOGCLASSIFIER_H
