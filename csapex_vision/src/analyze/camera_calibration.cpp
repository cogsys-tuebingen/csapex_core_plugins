/// HEADER
#include "camera_calibration.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::CameraCalibration, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

csapex::CameraCalibration::CameraCalibration() :
    update_request_(true)
{
}

void csapex::CameraCalibration::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

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
    msg::publish(output_, out);

}

void csapex::CameraCalibration::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("image");
    output_ = node_modifier.addOutput<CvMatMessage>("rendered corners");
}

void csapex::CameraCalibration::add()
{
    calibration_->addFrame();
}

void csapex::CameraCalibration::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareFileOutputPath("results", ""));
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("add"),
                            std::bind(&CameraCalibration::add, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("reset"),
                            std::bind(&CameraCalibration::requestUpdateCalibration, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("calibrate"),
                            std::bind(&CameraCalibration::calibrate, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("kernel", 1, 31, 11, 2),
                            std::bind(&CameraCalibration::requestUpdateCalibration, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("squares x", 1, 12, 5, 1),
                            std::bind(&CameraCalibration::requestUpdateCalibration, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("squares y", 1, 12, 8, 1),
                            std::bind(&CameraCalibration::requestUpdateCalibration, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("squares scale", 0.05, 0.5, 0.1, 0.05),
                            std::bind(&CameraCalibration::requestUpdateCalibration, this));

    std::map<std::string, int> types {
        {"chessboard", (int) cslibs_vision::CameraCalibration::CHESSBOARD},
        {"circles grid", (int) cslibs_vision::CameraCalibration::CIRCLES_GRID},
        {"asym. circles grid", (int) cslibs_vision::CameraCalibration::ASYMMETRIC_CIRCLES_GRID}
    };

    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet<int>("type", types, (int) cslibs_vision::CameraCalibration::CHESSBOARD),
                            std::bind(&CameraCalibration::requestUpdateCalibration, this));

    std::map<std::string, std::pair<int, bool> > corner_flags = {
        {"CV_CALIB_CB_ADAPTIVE_THRESH", std::make_pair(CV_CALIB_CB_ADAPTIVE_THRESH, true)},
        {"CV_CALIB_CB_FAST_CHECK",      std::make_pair(CV_CALIB_CB_FAST_CHECK, false)},
        {"CV_CALIB_CB_NORMALIZE_IMAGE", std::make_pair(CV_CALIB_CB_NORMALIZE_IMAGE, false)},
        {"CV_CALIB_CB_FILTER_QUADS",    std::make_pair(CV_CALIB_CB_FILTER_QUADS, true)}
    };
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterBitSet("corner flags", corner_flags),
                            std::bind(&CameraCalibration::requestUpdateCalibration, this));

    std::map<std::string, std::pair<int, bool> > calib_flags = {
        {"CV_CALIB_FIX_K4", std::make_pair(CV_CALIB_FIX_K4, true)},
        {"CV_CALIB_FIX_K5", std::make_pair(CV_CALIB_FIX_K5, true)},
        {"C_CALIB_FIX_K6", std::make_pair(CV_CALIB_FIX_K6, false)}
    };
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterBitSet("calib flags", calib_flags),
                            std::bind(&CameraCalibration::requestUpdateCalibration, this));
}

void csapex::CameraCalibration::calibrate()
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

void csapex::CameraCalibration::updateCalibration()
{
    cv::Size board_size;
    cslibs_vision::CameraCalibration::Mode mode =
            (cslibs_vision::CameraCalibration::Mode) readParameter<int>("type");
    board_size.width   = readParameter<int>("squares x");
    board_size.height  = readParameter<int>("squares y");
    double square_size = readParameter<double>("squares scale");
    int    kernel_size = readParameter<int>("kernel");
    int    flag_corner = readParameter<int>("corner flags");
    int    flag_calib  = readParameter<int>("calib flags");
    calibration_.reset(new cslibs_vision::CameraCalibration(mode, board_size, square_size, kernel_size, flag_corner, flag_calib));
}

void csapex::CameraCalibration::requestUpdateCalibration()
{
    update_request_ = true;
}
