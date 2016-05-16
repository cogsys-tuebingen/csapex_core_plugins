/// HEADER
#include "render_rois.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM


CSAPEX_REGISTER_CLASS(csapex::RenderROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;

RenderROIs::RenderROIs()
{
}

void RenderROIs::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<int>("thickness", 1, 20, 1, 1));
    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("color", 0,0,0));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("force color", false));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("ignore unclassified", false));
}

void RenderROIs::process()
{
    CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(input_img_);

    CvMatMessage::Ptr out(new CvMatMessage(img->getEncoding(), img->stamp_micro_seconds));

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
    if(msg::hasMessage(input_rois_)) {
        VectorMessage::ConstPtr rois = msg::getMessage<VectorMessage>(input_rois_);
        for(const TokenData::ConstPtr& e : rois->value) {
            RoiMessage::ConstPtr roi = std::dynamic_pointer_cast<RoiMessage const>(e);

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

    if(msg::hasMessage(input_rois_gen_)) {
        std::shared_ptr< std::vector<RoiMessage> const> rois =
                msg::getMessage<GenericVectorMessage, RoiMessage>(input_rois_gen_);

        for(const RoiMessage& roi : *rois) {
            if(ignore_uc && roi.value.classification() == -1) {
                continue;
            }

            cv::rectangle(out->value, roi.value.rect(), force_color ? color : roi.value.color(), thickness);

            std::string text = roi.value.label();

            if(!text.empty()) {
                cv::Point pt = roi.value.rect().tl();
                cv::putText(out->value, text, pt, cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar::all(0), 4, CV_AA);
                cv::putText(out->value, text, pt, cv::FONT_HERSHEY_SIMPLEX, 1., roi.value.color(), 1, CV_AA);
            }
        }
    }

    msg::publish(output_, out);
}

void RenderROIs::setup(NodeModifier& node_modifier)
{
    input_img_      = node_modifier.addInput<CvMatMessage>("image");
    input_rois_     = node_modifier.addOptionalInput<VectorMessage, RoiMessage>("ROIs");
    input_rois_gen_ = node_modifier.addOptionalInput<GenericVectorMessage, RoiMessage>("ROIs");

    output_ = node_modifier.addOutput<CvMatMessage>("Rendered Image");
}
