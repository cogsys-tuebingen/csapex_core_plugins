/// HEADER
#include "extract_roi.h"

/// PROJECT
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM


CSAPEX_REGISTER_CLASS(csapex::ExtractROI, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExtractROI::ExtractROI()
{
}

void ExtractROI::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<int>("thickness", 1, 20, 1, 1));
}

void ExtractROI::process()
{
    CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(input_img_);
    RoiMessage::ConstPtr roi = msg::getMessage<RoiMessage>(input_roi_);

    CvMatMessage::Ptr out(new CvMatMessage(img->getEncoding(), img->frame_id, img->stamp_micro_seconds));

    cv::Mat mat = img->value;
    cv::Rect rect = roi->value.rect();

    if(rect.x < 0) {
        rect.x = 0;
    } else if(rect.x >= mat.cols) {
        rect.x = mat.cols;
    }
    if(rect.y < 0) {
        rect.y = 0;
    } else if(rect.y >= mat.rows) {
        rect.y = mat.rows;
    }

    if(rect.x + rect.width >= mat.cols) {
        rect.width = mat.cols - rect.x - 1;
    }
    if(rect.y + rect.height >= mat.rows) {
        rect.height = mat.rows - rect.y - 1;
    }

    cv::Mat(mat, rect).copyTo(out->value);

    msg::publish(output_, out);
}

void ExtractROI::setup(NodeModifier& node_modifier)
{
    input_img_ = node_modifier.addInput<CvMatMessage>("Image");
    input_roi_ = node_modifier.addInput<RoiMessage >("ROI");

    output_ = node_modifier.addOutput<CvMatMessage>("SubImage");
}
