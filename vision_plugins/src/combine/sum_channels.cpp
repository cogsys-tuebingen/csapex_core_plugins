/// HEADER
#include "sum_channels.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(vision_plugins::SumChannels, csapex::Node)

using namespace csapex::connection_types;
using namespace csapex;
using namespace vision_plugins;

SumChannels::SumChannels()
{
}

void SumChannels::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    out->value = cv::Mat(in->value.rows, in->value.cols, CV_32F, 0.f);
    std::vector<cv::Mat> channels;
    cv::split(in->value, channels);
    bool abs = readParameter<bool>("abs");
    for(std::vector<cv::Mat>::iterator it = channels.begin() ; it != channels.end() ; ++it) {
        it->convertTo(*it, CV_32FC1);
        if(abs)
            *it = cv::abs(*it);
        cv::add(*it, out->value, out->value);
    }
    if(readParameter<bool>("mean")) {
       out->value = out->value * (1.0 / (double) channels.size());
    }
    msg::publish(output_, out);
}

void SumChannels::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("sum");
}

void SumChannels::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("mean", true));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("abs",  false));
}
