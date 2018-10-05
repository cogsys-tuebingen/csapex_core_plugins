/// HEADER
#include "camera_calibration.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::CameraCalibration, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

csapex::CameraCalibration::CameraCalibration() : accepted_(0), rejected_(0)
{
}

void csapex::CameraCalibration::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(input_);
    if (in->value.empty()) {
        throw std::runtime_error("Cannot run UndistortRectify on empty left image!");
    }
    if (!in->getEncoding().matches(enc::mono) && !in->getEncoding().matches(enc::bgr)) {
        throw std::runtime_error("Encoding must be 'mono' or 'bgr'!");
    }

    const cv::Mat& image = in->value;

    if (image_size_.height == 0 || image_size_.width == 0) {
        image_size_.height = image.rows;
        image_size_.width = image.cols;
    } else if (image_size_ != cv::Size(image.cols, image.rows)) {
        resetPoints();
    }

    /// auto add corners ... if found /// according to checker board size
    /// 1. find corners
    bool found = true;
    std::vector<cv::Point2f> corners;
    switch (pattern_type_) {
        case CHESSBOARD:
            found = cv::findChessboardCorners(image, pattern_size_, corners, corner_flags_);
            if (found) {
                if (image.type() == CV_8UC3) {
                    cv::Mat gray;
                    cv::cvtColor(image, gray, CV_BGR2GRAY);
                    cv::cornerSubPix(gray, corners, cv::Size(kernel_size_, kernel_size_), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                } else {
                    cv::cornerSubPix(image, corners, cv::Size(kernel_size_, kernel_size_), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                }
            }
            break;
        case CIRCLES_GRID:
            found = cv::findCirclesGrid(image, pattern_size_, corners);
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            found = cv::findCirclesGrid(image, pattern_size_, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            throw std::runtime_error("Unknown pattern type!");
    }
    /// 2. render corners found
    CvMatMessage::Ptr out_corners(new CvMatMessage(enc::bgr, in->frame_id, in->stamp_micro_seconds));
    cv::cvtColor(image, out_corners->value, CV_GRAY2BGR);
    cv::drawChessboardCorners(out_corners->value, pattern_size_, corners, found);
    msg::publish(output_corners_, out_corners);
    /// 3. add corners to detected queue
    if (found) {
        std::vector<cv::Point3f> object_points;
        switch (pattern_type_) {
            case CHESSBOARD:
            case CIRCLES_GRID:
                for (int i = 0; i < pattern_size_.height; ++i) {
                    for (int j = 0; j < pattern_size_.width; ++j) {
                        object_points.emplace_back(cv::Point3f(pattern_scale_ * j, pattern_scale_ * i, 0.f));
                    }
                }
                break;
            case ASYMMETRIC_CIRCLES_GRID:
                for (int i = 0; i < pattern_size_.height; ++i) {
                    for (int j = 0; j < pattern_size_.width; ++j) {
                        object_points.emplace_back(cv::Point3f(pattern_scale_ * (2 * j + i % 2), pattern_scale_ * i, 0.f));
                    }
                }
                break;
            default:
                break;
        }
        image_points_.emplace_back(corners);
        object_points_.emplace_back(object_points);
        ++accepted_;
    } else {
        ++rejected_;
    }

    msg::publish(output_accepted_, accepted_);
    msg::publish(output_rejected_, rejected_);
}

void csapex::CameraCalibration::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Image");
    output_corners_ = node_modifier.addOutput<CvMatMessage>("Corners");
    output_accepted_ = node_modifier.addOutput<int>("accepted");
    output_rejected_ = node_modifier.addOutput<int>("rejected");
}

void csapex::CameraCalibration::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareFileOutputPath("path_calibration", "", "*.yaml"), path_calibration_);

    parameters.addParameter(csapex::param::factory::declareTrigger("reset"), std::bind(&CameraCalibration::resetPoints, this));
    parameters.addParameter(csapex::param::factory::declareTrigger("calibrate"), std::bind(&CameraCalibration::calibrate, this));

    parameters.addParameter(csapex::param::factory::declareRange("kernel", 1, 31, 11, 2), std::bind(&CameraCalibration::updateParameters, this));
    parameters.addParameter(csapex::param::factory::declareRange("squares_x", 1, 12, 5, 1), std::bind(&CameraCalibration::updateParameters, this));
    parameters.addParameter(csapex::param::factory::declareRange("squares_y", 1, 12, 8, 1), std::bind(&CameraCalibration::updateParameters, this));
    parameters.addParameter(csapex::param::factory::declareRange("squares_scale", 0.05, 0.5, 0.1, 0.05), std::bind(&CameraCalibration::updateParameters, this));

    std::map<std::string, int> types{ { "chessboard", static_cast<int>(CHESSBOARD) },
                                      { "circles grid", static_cast<int>(CIRCLES_GRID) },
                                      { "asym. circles grid", static_cast<int>(ASYMMETRIC_CIRCLES_GRID) } };

    parameters.addParameter(csapex::param::factory::declareParameterSet<int>("pattern_type", types, static_cast<int>(CHESSBOARD)), std::bind(&CameraCalibration::updateParameters, this));

    std::map<std::string, std::pair<int, bool>> corner_flags = { { "CV_CALIB_CB_ADAPTIVE_THRESH", std::make_pair(CV_CALIB_CB_ADAPTIVE_THRESH, true) },
                                                                 { "CV_CALIB_CB_FAST_CHECK", std::make_pair(CV_CALIB_CB_FAST_CHECK, false) },
                                                                 { "CV_CALIB_CB_NORMALIZE_IMAGE", std::make_pair(CV_CALIB_CB_NORMALIZE_IMAGE, false) },
                                                                 { "CV_CALIB_CB_FILTER_QUADS", std::make_pair(CV_CALIB_CB_FILTER_QUADS, true) } };
    parameters.addParameter(csapex::param::factory::declareParameterBitSet("corner_flags", corner_flags), std::bind(&CameraCalibration::updateParameters, this));

    std::map<std::string, std::pair<int, bool>> calib_flags = { { "CV_CALIB_FIX_K4", std::make_pair(CV_CALIB_FIX_K4, true) },
                                                                { "CV_CALIB_FIX_K5", std::make_pair(CV_CALIB_FIX_K5, true) },
                                                                { "C_CALIB_FIX_K6", std::make_pair(CV_CALIB_FIX_K6, false) } };
    parameters.addParameter(csapex::param::factory::declareParameterBitSet("calibration_flags", calib_flags), std::bind(&CameraCalibration::updateParameters, this));
}

void CameraCalibration::calibrate()
{
    cv::Mat intrinsics;
    cv::Mat distortions;
    std::vector<cv::Mat> rvecs, tvecs;
    ainfo << "started calibration" << std::endl;
    cv::calibrateCamera(object_points_, image_points_, image_size_, intrinsics, distortions, rvecs, tvecs, calibration_flags_);
    ainfo << "finished calibration" << std::endl;

    if (path_calibration_ == "") {
        throw std::runtime_error("Path may not be empty to save the calibration!");
    }

    cv::FileStorage fs(path_calibration_, cv::FileStorage::WRITE);
    fs << "intrinsics" << intrinsics;
    fs << "distortion" << distortions;
    fs.release();
}

void CameraCalibration::updateParameters()
{
    kernel_size_ = readParameter<int>("kernel");
    pattern_size_.width = readParameter<int>("squares_x");
    pattern_size_.height = readParameter<int>("squares_y");
    pattern_scale_ = readParameter<double>("squares_scale");
    pattern_type_ = readParameter<int>("pattern_type");
    corner_flags_ = readParameter<int>("corner_flags");
    calibration_flags_ = readParameter<int>("calibration_flags");
    reset();
}

void CameraCalibration::resetPoints()
{
    image_points_.clear();
    object_points_.clear();

    accepted_ = 0;
    rejected_ = 0;
}
