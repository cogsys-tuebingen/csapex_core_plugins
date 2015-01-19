/// HEADER
#include "camera_calibration.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/assign/std.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::CameraCalibration, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

vision_plugins::CameraCalibration::CameraCalibration() :
    update_request_(true)
{
}

void vision_plugins::CameraCalibration::process()
{
    CvMatMessage::ConstPtr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    /// BUFFER THE CURRENT FRAME
    buffer_frame_ = in->value.clone();

    if(update_request_) {
        updateCalibration();
        update_request_ = false;
    }

    calibration_->analyze(buffer_frame_);

    /// OUTPUT
    if(in->hasChannels(1, CV_8U)) {
        cv::cvtColor(in->value, out->value, CV_GRAY2BGR);
        out->setEncoding(enc::bgr);
    } else {
        out->value = in->value.clone();
    }

    calibration_->drawFoundCorners(out->value);
    output_->publish(out);

}

void vision_plugins::CameraCalibration::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("image");
    output_ = modifier_->addOutput<CvMatMessage>("rendered corners");
}

void vision_plugins::CameraCalibration::add()
{
    calibration_->addFrame();
}

void vision_plugins::CameraCalibration::setupParameters()
{
    addParameter(param::ParameterFactory::declareDirectoryOutputPath("results", ""));
    addParameter(param::ParameterFactory::declareTrigger("add"),
                 std::bind(&CameraCalibration::add, this));
    addParameter(param::ParameterFactory::declareTrigger("reset"),
                 std::bind(&CameraCalibration::requestUpdateCalibration, this));
    addParameter(param::ParameterFactory::declareTrigger("calibrate"),
                 std::bind(&CameraCalibration::calibrate, this));
    addParameter(param::ParameterFactory::declareRange("kernel", 1, 31, 11, 2),
                 std::bind(&CameraCalibration::requestUpdateCalibration, this));
    addParameter(param::ParameterFactory::declareRange("squares x", 1, 12, 5, 1),
                 std::bind(&CameraCalibration::requestUpdateCalibration, this));
    addParameter(param::ParameterFactory::declareRange("squares y", 1, 12, 8, 1),
                 std::bind(&CameraCalibration::requestUpdateCalibration, this));
    addParameter(param::ParameterFactory::declareRange("squares scale", 0.05, 0.5, 0.1, 0.05),
                 std::bind(&CameraCalibration::requestUpdateCalibration, this));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("chessboard", (int) utils_vision::CameraCalibration::CHESSBOARD)
            ("circles grid", (int) utils_vision::CameraCalibration::CIRCLES_GRID)
            ("asym. circles grid", (int) utils_vision::CameraCalibration::ASYMMETRIC_CIRCLES_GRID);

    addParameter(param::ParameterFactory::declareParameterSet<int>("type", types, (int) utils_vision::CameraCalibration::CHESSBOARD),
                 std::bind(&CameraCalibration::requestUpdateCalibration, this));

    std::map<std::string, std::pair<int, bool> > corner_flags = boost::assign::map_list_of
            ("CV_CALIB_CB_ADAPTIVE_THRESH", std::make_pair(CV_CALIB_CB_ADAPTIVE_THRESH, true))
            ("CV_CALIB_CB_FAST_CHECK",      std::make_pair(CV_CALIB_CB_FAST_CHECK, false))
            ("CV_CALIB_CB_NORMALIZE_IMAGE", std::make_pair(CV_CALIB_CB_NORMALIZE_IMAGE, false))
            ("CV_CALIB_CB_FILTER_QUADS",    std::make_pair(CV_CALIB_CB_FILTER_QUADS, true));
    addParameter(param::ParameterFactory::declareParameterBitSet("corner flags", corner_flags),
                 std::bind(&CameraCalibration::requestUpdateCalibration, this));

    std::map<std::string, std::pair<int, bool> > calib_flags = boost::assign::map_list_of
            ("CV_CALIB_FIX_K4", std::make_pair(CV_CALIB_FIX_K4, true))
            ("CV_CALIB_FIX_K5", std::make_pair(CV_CALIB_FIX_K5, true))
            ("CV_CALIB_FIX_K6", std::make_pair(CV_CALIB_FIX_K6, false));
    addParameter(param::ParameterFactory::declareParameterBitSet("calib flags", calib_flags),
                 std::bind(&CameraCalibration::requestUpdateCalibration, this));
}

void vision_plugins::CameraCalibration::calibrate()
{
    std::string path = readParameter<std::string>("results");
    if(path == "") {
        aerr << "Cannot save to empty path!" << std::endl;
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

void vision_plugins::CameraCalibration::updateCalibration()
{
    cv::Size board_size;
    utils_vision::CameraCalibration::Mode mode =
            (utils_vision::CameraCalibration::Mode) readParameter<int>("type");
    board_size.width   = readParameter<int>("squares x");
    board_size.height  = readParameter<int>("squares y");
    double square_size = readParameter<double>("squares scale");
    int    kernel_size = readParameter<int>("kernel");
    int    flag_corner = readParameter<int>("corner flags");
    int    flag_calib  = readParameter<int>("calib flags");
    calibration_.reset(new utils_vision::CameraCalibration(mode, board_size, square_size, kernel_size, flag_corner, flag_calib));
}

void vision_plugins::CameraCalibration::requestUpdateCalibration()
{
    update_request_ = true;
}
