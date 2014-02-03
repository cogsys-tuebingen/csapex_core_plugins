#ifndef FILTER_UNDISTORT_H
#define FILTER_UNDISTORT_H

/// COMPONENT
#include <utils_cv/undistortion.h>
#include <csapex/model/node.h>

namespace csapex {
class Undistort : public csapex::Node
{
public:
    Undistort();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    utils_cv::Undistortion::Ptr undist_;

    bool read_matrices(const std::string &path, cv::Mat &intrinsics, cv::Mat &distortion_coeffs);
    void updateUndistorter();

    ConnectorOut* output_;
    ConnectorIn* input_;
};

}

#endif // FILTER_UNDISTORT_H
