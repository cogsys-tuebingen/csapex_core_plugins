/// HEADER
#include "camera_calibration.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <boost/assign/std.hpp>

CSAPEX_REGISTER_CLASS(csapex::CameraCalibration, csapex::Node)

using namespace csapex::connection_types;

csapex::CameraCalibration::CameraCalibration()
{
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declarePath("results", ""));
    addParameter(param::ParameterFactory::declareTrigger("add"), boost::bind(&CameraCalibration::add, this));
    addParameter(param::ParameterFactory::declareTrigger("reset"), boost::bind(&CameraCalibration::updateCalibration, this));
    addParameter(param::ParameterFactory::declareTrigger("calibrate"), boost::bind(&CameraCalibration::calibrate, this));
    addParameter(param::ParameterFactory::declareRange("kernel", 1, 31, 11, 2), boost::bind(&CameraCalibration::updateCalibration, this));
    addParameter(param::ParameterFactory::declareRange("squares x", 1, 12, 5, 1), boost::bind(&CameraCalibration::updateCalibration, this));
    addParameter(param::ParameterFactory::declareRange("squares y", 1, 12, 8, 1), boost::bind(&CameraCalibration::updateCalibration, this));
    addParameter(param::ParameterFactory::declareRange("squares scale", 0.05, 0.5, 0.1, 0.05), boost::bind(&CameraCalibration::updateCalibration, this));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("chessboard", (int) utils_cv::CameraCalibration::CHESSBOARD)
            ("circles grid", (int) utils_cv::CameraCalibration::CIRCLES_GRID)
            ("asym. circles grid", (int) utils_cv::CameraCalibration::ASYMMETRIC_CIRCLES_GRID);

    addParameter(param::ParameterFactory::declareParameterSet<int>("type", types), boost::bind(&CameraCalibration::updateCalibration, this));

    std::map<std::string, int> corner_flags = boost::assign::map_list_of
            ("CV_CALIB_CB_ADAPTIVE_THRESH", CV_CALIB_CB_ADAPTIVE_THRESH)
            ("CV_CALIB_CB_FAST_CHECK",      CV_CALIB_CB_FAST_CHECK)
            ("CV_CALIB_CB_NORMALIZE_IMAGE", CV_CALIB_CB_NORMALIZE_IMAGE)
            ("CV_CALIB_CB_FILTER_QUADS",    CV_CALIB_CB_FILTER_QUADS);

    std::map<std::string, int> calib_flags = boost::assign::map_list_of
            ("CV_CALIB_FIX_K4", CV_CALIB_FIX_K4)
            ("CV_CALIB_FIX_K5", CV_CALIB_FIX_K5)
            ("CV_CALIB_FIX_K6", CV_CALIB_FIX_K6);


}

void csapex::CameraCalibration::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    /// BUFFER THE CURRENT FRAME
    buffer_frame_ = in->value.clone();

    calibration_->analyze(buffer_frame_);

    /// OUTPUT
    if(in->getEncoding().size() == 1) {
        cv::cvtColor(in->value, out->value, CV_GRAY2BGR);
        out->setEncoding(enc::bgr);
    } else {
        out->value = in->value.clone();
    }

    calibration_->drawFoundCorners(out->value);
    output_->publish(out);

}

void csapex::CameraCalibration::setup()
{
    setSynchronizedInputs(true);

    input_  = addInput<CvMatMessage>("Image");
    output_ = addOutput<CvMatMessage>("Rendered Corners");
    updateCalibration();
}

void csapex::CameraCalibration::add()
{
    calibration_->addFrame();
}

void csapex::CameraCalibration::calibrate()
{
    std::string path = param<std::string>("results");
    if(path == "") {
        std::cerr << "Cannot save to empty path!" << std::endl;
        return;
    }

    cv::Mat intrinsics;
    cv::Mat distortions;
    calibration_->calibrate(intrinsics, distortions);

    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    fs << "intrinsics" << intrinsics;
    fs << "distortion" << distortions;
    fs.release();
}

void csapex::CameraCalibration::updateCalibration()
{
    cv::Size board_size;
    utils_cv::CameraCalibration::Mode mode = (utils_cv::CameraCalibration::Mode) param<int>("type");
    board_size.width  = param<int>("squares x");
    board_size.height = param<int>("squares y");
    double square_size = param<double>("squares scale");
    int    kernel_size = param<int>("kernel");
    calibration_.reset(new utils_cv::CameraCalibration(mode, board_size, square_size, kernel_size));
}
