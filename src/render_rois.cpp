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
    addParameter(param::ParameterFactory::declareColorParameter("color", 0,0,0));
    addParameter(param::ParameterFactory::declareBool("force color", false));
}

void RenderROIs::allConnectorsArrived()
{
    CvMatMessage::Ptr img = input_img_->getMessage<CvMatMessage>();
    VectorMessage::Ptr rois = input_rois_->getMessage<VectorMessage>();

    CvMatMessage::Ptr out(new CvMatMessage(img->getEncoding()));

    int thickness = param<int>("thickness");
    bool force_color = param<bool>("force color");

    if(img->getEncoding().size() == 1) {
        cv::cvtColor(img->value, out->value, CV_GRAY2BGR);
        out->setEncoding(enc::bgr);
    } else {
        img->value.copyTo(out->value);
    }

    cv::Scalar color;
    if(force_color) {
        std::vector<int> c = param<std::vector<int> >("color");
        color = cv::Scalar(c[2], c[1], c[0]);
    }

    BOOST_FOREACH(const ConnectionType::Ptr& e, rois->value) {
        RoiMessage::Ptr roi = boost::dynamic_pointer_cast<RoiMessage>(e);
        cv::rectangle(out->value, roi->value.rect(), force_color ? color : roi->value.color(), thickness);

        std::string text = roi->value.label();

        if(!text.empty()) {
            cv::Point pt = roi->value.rect().tl();
            cv::putText(out->value, text, pt, cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar::all(0), 4, CV_AA);
            cv::putText(out->value, text, pt, cv::FONT_HERSHEY_SIMPLEX, 1., roi->value.color(), 1, CV_AA);
        }
    }

    output_->publish(out);
}

void RenderROIs::setup()
{
    setSynchronizedInputs(true);

    input_img_ = addInput<CvMatMessage>("Image");
    input_rois_ = addInput<VectorMessage, RoiMessage>("ROIs");

    output_ = addOutput<CvMatMessage>("Rendered Image");
}
