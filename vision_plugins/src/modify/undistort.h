#ifndef FILTER_UNDISTORT_H
#define FILTER_UNDISTORT_H

/// COMPONENT
#include <utils_cv/undistortion.h>
#include <csapex/model/node.h>

namespace vision_plugins {
class Undistort : public csapex::Node
{
public:
    Undistort();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:
    utils_cv::Undistortion::Ptr undist_;

    bool read_matrices(const std::string &path, cv::Mat &intrinsics, cv::Mat &distortion_coeffs);
    void update();

    csapex::ConnectorOut* output_;
    csapex::ConnectorIn* input_;
};

}

#endif // FILTER_UNDISTORT_H
