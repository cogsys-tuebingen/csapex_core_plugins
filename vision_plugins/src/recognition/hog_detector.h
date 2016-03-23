#ifndef HOG_DETECT_H
#define HOG_DETECT_H

/// PROJECT
#include <csapex/model/node.h>
#include "hog.h"

namespace vision_plugins {
class HOGDetector : public csapex::Node
{
public:
    HOGDetector();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    enum SVMType  {DEFAULT = 0, DAIMLER, CUSTOM};
    enum ScanMode {SINGLE_SCALE = 0, MULTI_SCALE};

    csapex::Input*  in_;
    csapex::Output* out_;

    HOGDescriptor         hog_;
    int                   hog_win_width_;
    int                   hog_win_height_;
    int                   scan_mode_;
    std::vector<float>    svm_;
    int                   svm_type_;
    double                svm_thresh_;

    void load();
};
}

#endif // HOG_DETECT_H
