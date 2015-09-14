/// HEADER
#include "number_generator.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::NumberGenerator, csapex::Node)

using namespace csapex;

NumberGenerator::NumberGenerator()
    : n(0)
{
}

void NumberGenerator::process()
{
    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(enc::bgr, 0));

    msg->value = cv::Mat(400, 400, CV_8UC3);

    std::stringstream txt;
    txt << n;
    cv::rectangle(msg->value, cv::Rect(0,0, msg->value.cols, msg->value.rows), cv::Scalar::all(0), CV_FILLED);
    cv::putText(msg->value, txt.str(), cv::Point(100, 200), CV_FONT_HERSHEY_PLAIN, 5.0, cv::Scalar::all(255), 2, CV_AA);

    msg::publish(output_, msg);

    ++n;
}

void NumberGenerator::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::AnyMessage>("Trigger");
    output_ = node_modifier.addOutput<connection_types::CvMatMessage>("Image");
}
