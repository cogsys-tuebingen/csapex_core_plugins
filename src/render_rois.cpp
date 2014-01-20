/// HEADER
#include "render_rois.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/foreach.hpp>

CSAPEX_REGISTER_CLASS(csapex::RenderROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;

RenderROIs::RenderROIs()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("ROI"));

    addParameter(param::ParameterFactory::declare<int>("thickness", 1, 20, 1, 1));
}

void RenderROIs::allConnectorsArrived()
{
    CvMatMessage::Ptr img = input_img_->getMessage<CvMatMessage>();
    VectorMessage::Ptr rois = input_rois_->getMessage<VectorMessage>();

    CvMatMessage::Ptr out(new CvMatMessage);

    int thickness = param<int>("thickness");

    if(img->encoding.size() == 1) {
        cv::cvtColor(img->value, out->value, CV_GRAY2BGR);
    } else {
        img->value.copyTo(out->value);
    }

    BOOST_FOREACH(const ConnectionType::Ptr& e, rois->value) {
        RoiMessage::Ptr roi = boost::dynamic_pointer_cast<RoiMessage>(e);
        cv::rectangle(out->value, roi->value.rect(), roi->value.color(), thickness);
    }

    output_->publish(out);
}

void RenderROIs::setup()
{
    setSynchronizedInputs(true);

    input_img_ = addInput<CvMatMessage>("Image");
    input_rois_ = addInput<VectorMessage>("ROIs");
    input_rois_->setType(VectorMessage::Ptr(new VectorMessage(RoiMessage::make())));

    output_ = addOutput<CvMatMessage>("Rendered Image");
}
