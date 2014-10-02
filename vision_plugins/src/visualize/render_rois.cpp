/// HEADER
#include "render_rois.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/foreach.hpp>

CSAPEX_REGISTER_CLASS(csapex::RenderROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;

RenderROIs::RenderROIs()
{
}

void RenderROIs::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange<int>("thickness", 1, 20, 1, 1));
    addParameter(param::ParameterFactory::declareColorParameter("color", 0,0,0));
    addParameter(param::ParameterFactory::declareBool("force color", false));
    addParameter(param::ParameterFactory::declareBool("ignore unclassified", false));
}

void RenderROIs::process()
{
    CvMatMessage::Ptr img = input_img_->getMessage<CvMatMessage>();

    CvMatMessage::Ptr out(new CvMatMessage(img->getEncoding(), img->stamp));

    int thickness = readParameter<int>("thickness");
    bool force_color = readParameter<bool>("force color");

    if(img->hasChannels(1, CV_8U)) {
        cv::cvtColor(img->value, out->value, CV_GRAY2BGR);
        out->setEncoding(enc::bgr);
    } else {
        img->value.copyTo(out->value);
    }

    cv::Scalar color;
    if(force_color) {
        std::vector<int> c = readParameter<std::vector<int> >("color");
        color = cv::Scalar(c[2], c[1], c[0]);
    }

    bool ignore_uc = readParameter<bool>("ignore unclassified");
    if(input_rois_->hasMessage()) {
        VectorMessage::Ptr rois = input_rois_->getMessage<VectorMessage>();
        BOOST_FOREACH(const ConnectionType::Ptr& e, rois->value) {
            RoiMessage::Ptr roi = boost::dynamic_pointer_cast<RoiMessage>(e);

            if(ignore_uc && roi->value.classification() == -1) {
                continue;
            }

            cv::rectangle(out->value, roi->value.rect(), force_color ? color : roi->value.color(), thickness);

            std::string text = roi->value.label();

            if(!text.empty()) {
                cv::Point pt = roi->value.rect().tl();
                cv::putText(out->value, text, pt, cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar::all(0), 4, CV_AA);
                cv::putText(out->value, text, pt, cv::FONT_HERSHEY_SIMPLEX, 1., roi->value.color(), 1, CV_AA);
            }
        }
    }

    if(input_rois_gen_->hasMessage()) {
        boost::shared_ptr< std::vector<RoiMessage::Ptr> const> rois =
                input_rois_gen_->getMessage<GenericVectorMessage, RoiMessage::Ptr>();

        BOOST_FOREACH(const RoiMessage::Ptr& roi, *rois) {
            if(ignore_uc && roi->value.classification() == -1) {
                continue;
            }

            cv::rectangle(out->value, roi->value.rect(), force_color ? color : roi->value.color(), thickness);

            std::string text = roi->value.label();

            if(!text.empty()) {
                cv::Point pt = roi->value.rect().tl();
                cv::putText(out->value, text, pt, cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar::all(0), 4, CV_AA);
                cv::putText(out->value, text, pt, cv::FONT_HERSHEY_SIMPLEX, 1., roi->value.color(), 1, CV_AA);
            }
        }
    }

    output_->publish(out);
}

void RenderROIs::setup()
{
    input_img_      = modifier_->addInput<CvMatMessage>("image");
    input_rois_     = modifier_->addOptionalInput<VectorMessage, RoiMessage>("ROIs");
    input_rois_gen_ = modifier_->addOptionalInput<GenericVectorMessage, RoiMessage::Ptr>("ROIs");

    output_ = modifier_->addOutput<CvMatMessage>("Rendered Image");
}
