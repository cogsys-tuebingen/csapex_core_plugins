/// HEADER
#include "camera_calibration.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::CameraCalibration, csapex::Node)

using namespace csapex::connection_types;

csapex::CameraCalibration::CameraCalibration()
{
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declarePath("results", ""));
    addParameter(param::ParameterFactory::declareTrigger("add"));
    addParameter(param::ParameterFactory::declareTrigger("reset"));
    addParameter(param::ParameterFactory::declareTrigger("calibrate"));
    addParameter(param::ParameterFactory::declareRange("squares x", 1, 12, 5, 1));
    addParameter(param::ParameterFactory::declareRange("squares y", 1, 12, 8, 1));
    addParameter(param::ParameterFactory::declareRange("squares scale", 0.05, 0.5, 0.1, 0.05));

}

void csapex::CameraCalibration::allConnectorsArrived()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage);

    /// BUFFER THE CURRENT FRAME
    buffer_frame_ = in->value.clone();

}

void csapex::CameraCalibration::setup()
{
    setSynchronizedInputs(true);

    input_  = addInput<CvMatMessage>("Image");
    output_ = addOutput<CvMatMessage>("Rendered Corners");
}

void csapex::CameraCalibration::add()
{

}

void csapex::CameraCalibration::reset()
{

}

void csapex::CameraCalibration::calibrate()
{

}

void csapex::CameraCalibration::updateCalibration()
{
//    calibration_.reset(new CameraCalibration());
}
