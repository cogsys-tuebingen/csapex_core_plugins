/// HEADER
#include "sum_channels.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(vision_plugins::SumChannels, csapex::Node)

using namespace csapex::connection_types;
using namespace csapex;
using namespace vision_plugins;

SumChannels::SumChannels()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));
    addParameter(param::ParameterFactory::declareBool("mean", true));
    addParameter(param::ParameterFactory::declareBool("abs",  false));
}

void SumChannels::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    out->value = cv::Mat(in->value.rows, in->value.cols, CV_32F, 0.f);
    std::vector<cv::Mat> channels;
    cv::split(in->value, channels);
    bool abs = param<bool>("abs");
    for(std::vector<cv::Mat>::iterator it = channels.begin() ; it != channels.end() ; ++it) {
        it->convertTo(*it, CV_32FC1);
        if(abs)
            *it = cv::abs(*it);
        cv::add(*it, out->value, out->value);
    }
    if(param<bool>("mean")) {
       out->value = out->value * (1.0 / (double) channels.size());
    }
    output_->publish(out);
}

void SumChannels::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("original");
    output_ = modifier_->addOutput<CvMatMessage>("sum");
}
