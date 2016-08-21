/// HEADER
#include "float_to_uchar.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::FloatToUchar, csapex::Node)

FloatToUchar::FloatToUchar()
{
}

void FloatToUchar::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->stamp_micro_seconds));

    std::vector<cv::Mat> channels;
    cv::split(in->value, channels);
    for(std::vector<cv::Mat>::iterator
        it = channels.begin() ;
        it!= channels.end() ;
        ++it) {

        cv::normalize(*it, *it, 0.0, 255.0, cv::NORM_MINMAX);
        it->convertTo(*it, CV_8UC1);
    }
    cv::merge(channels, out->value);
    msg::publish(output_, out);
}

void FloatToUchar::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("float");
    output_ = node_modifier.addOutput<CvMatMessage>("uchar");
}

void FloatToUchar::setupParameters(Parameterizable& parameters)
{
}

