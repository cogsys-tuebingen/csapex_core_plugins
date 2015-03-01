#ifndef HOG_DETECT_H
#define HOG_DETECT_H

/// PROJECT
#include <csapex/model/node.h>

namespace vision_plugins {
class HOGDetector : public csapex::Node
{
public:
    HOGDetector();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    enum SVMType        {DEFAULT, DAIMLER, CUSTOM};
    enum DetectionType  {SINGLE_SCALE, MULTI_SCALE};

    csapex::Input*  in_;
    csapex::Output* out_;

    std::vector<float>    svm_;
    int                   svm_width_;
    int                   svm_height_;

    void load();
};
}

#endif // HOG_DETECT_H
