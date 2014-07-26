/// HEADER
#include "extract_roi.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/foreach.hpp>

CSAPEX_REGISTER_CLASS(csapex::ExtractROI, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExtractROI::ExtractROI()
{
    addParameter(param::ParameterFactory::declare<int>("thickness", 1, 20, 1, 1));
}

void ExtractROI::process()
{
    CvMatMessage::Ptr img = input_img_->getMessage<CvMatMessage>();
    RoiMessage::Ptr roi = input_roi_->getMessage<RoiMessage>();

    CvMatMessage::Ptr out(new CvMatMessage(img->getEncoding()));

    cv::Mat(img->value, roi->value.rect()).copyTo(out->value);

    output_->publish(out);
}

void ExtractROI::setup()
{
    input_img_ = modifier_->addInput<CvMatMessage>("Image");
    input_roi_ = modifier_->addInput<RoiMessage >("ROI");

    output_ = modifier_->addOutput<CvMatMessage>("SubImage");
}
