#ifndef FILTER_UNDISTORT_H
#define FILTER_UNDISTORT_H

/// COMPONENT
#include <utils_vision/utils/undistortion.h>
#include <csapex/model/node.h>

namespace vision_plugins {
class Undistort : public csapex::Node
{
public:
    Undistort();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(csapex::Parameterizable& parameters) override;

private:
    utils_vision::Undistortion::Ptr undist_;

    bool read_matrices(const std::string &path, cv::Mat &intrinsics, cv::Mat &distortion_coeffs);
    void update();

    csapex::Output* output_;
    csapex::Input* input_;
};

}

#endif // FILTER_UNDISTORT_H
