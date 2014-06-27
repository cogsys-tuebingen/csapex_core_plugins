#ifndef CAMERA_CALIBRATION_PLUGIN_H
#define CAMERA_CALIBRATION_PLUGIN_H

/// COMPONENT
#include <utils_cv/camera_calibration.h>
#include <csapex/model/node.h>

namespace vision_plugins {
class CameraCalibration : public csapex::Node
{
public:
    CameraCalibration();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:
    utils_cv::CameraCalibration::Ptr calibration_;

    csapex::ConnectorOut*            output_;
    csapex::ConnectorIn*             input_;

    cv::Mat                          buffer_frame_;

    void add();
    void calibrate();
    void updateCalibration();
};
}
#endif // CAMERA_CALIBRATION_PLUGIN_H
