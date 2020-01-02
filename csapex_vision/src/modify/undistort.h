#ifndef FILTER_UNDISTORT_H
#define FILTER_UNDISTORT_H

/// COMPONENT
#include <csapex/model/node.h>
#include <cslibs_vision/utils/undistortion.h>

namespace csapex
{
class Undistort : public csapex::Node
{
public:
    Undistort();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(csapex::Parameterizable& parameters) override;

private:
    cslibs_vision::Undistortion::Ptr undist_;

    bool read_matrices(const std::string& path, cv::Mat& intrinsics, cv::Mat& distortion_coeffs);
    void update();

    csapex::Output* output_;
    csapex::Input* input_;
};

}  // namespace csapex

#endif  // FILTER_UNDISTORT_H
