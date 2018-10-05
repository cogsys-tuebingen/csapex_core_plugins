#ifndef HOG_DETECT_H
#define HOG_DETECT_H

/// PROJECT
#include "hog.h"
#include <csapex/model/node.h>

namespace csapex
{
class HOGDetector : public csapex::Node
{
public:
    HOGDetector();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    enum SVMType
    {
        DEFAULT = 0,
        DAIMLER,
        CUSTOM,
        NONE
    };
    enum ScanMode
    {
        SINGLE_SCALE = 0,
        MULTI_SCALE
    };
    enum ClassificationType
    {
        BACKGROUND = 0,
        HUMAN = 1,
        HUMAN_PART = 2,
        UNKNOWN = 3
    };

    csapex::Input* in_;
    csapex::Output* out_;

    HOGDescriptor hog_;
    SVMType prev_svm_type_;
    int n_bins_;
    bool signed_gradient_;
    int cells_x_;
    int cells_y_;
    int cell_size_;
    int block_size_;
    int block_stride_;

    int adaption_type_;
    int scan_mode_;
    std::vector<float> svm_;
    int svm_type_;
    double svm_thresh_;

    void load();
    void setParameters(const int cell_size, const int cells_x, const int cells_y, const int block_size, const int block_stride, const int bins, const bool signed_gradient);
};
}  // namespace csapex

#endif  // HOG_DETECT_H
