/// HEADER
#include "watershed.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::WaterShed, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

WaterShed::WaterShed()
{

}

void WaterShed::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::unknown, in->stamp_micro_seconds));

    cv::Mat marker_mask;
    switch(in->value.type()) {
    case CV_8UC3:
        cv::cvtColor(in->value, marker_mask, CV_BGR2GRAY);
        break;
    case CV_8UC1:
        marker_mask = in->value.clone();
        break;
    default:
        throw std::runtime_error("Need a 3 or 1 channel bgr image!");
    }

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(marker_mask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    if(!contours.empty()) {

    }


    msg::publish(output_, out);
}

void WaterShed::setup(NodeModifier &node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("image");
    output_ = node_modifier.addOutput<CvMatMessage>("clusters");
}

void WaterShed::setupParameters(Parameterizable &parameters)
{
}
