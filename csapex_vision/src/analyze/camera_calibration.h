#ifndef CAMERA_CALIBRATION_PLUGIN_H
#define CAMERA_CALIBRATION_PLUGIN_H

/// PROJECT
#include <csapex/model/node.h>

/// COMPONENT
#include <cslibs_vision/utils/camera_calibration.h>

namespace csapex {
class CameraCalibration : public csapex::Node
{
public:
    CameraCalibration();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Input  *input_;
    Output *output_corners_;
    Output *output_rejected_;
    Output *output_accepted_;


    std::vector<std::vector<cv::Point2f>> image_points_;
    std::vector<std::vector<cv::Point3f>> object_points_;

    std::string path_calibration_;
    cv::Size    image_size_;
    int         kernel_size_;
    cv::Size    pattern_size_;
    double      pattern_scale_;
    int         pattern_type_;
    int         corner_flags_;
    int         calibration_flags_;
    int         accepted_;
    int         rejected_;

    enum Mode {CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID};

    void calibrate();
    void updateParameters();
    void resetPoints();
};
}
#endif // CAMERA_CALIBRATION_PLUGIN_H
