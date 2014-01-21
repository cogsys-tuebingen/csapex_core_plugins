/// HEADER
#include "number_generator.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::NumberGenerator, csapex::Node)

using namespace csapex;

NumberGenerator::NumberGenerator()
    : n(0)
{
    addTag(Tag::get("Debug"));
}

void NumberGenerator::allConnectorsArrived()
{
    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage);

    msg->encoding = enc::bgr;
    msg->value = cv::Mat(400, 400, CV_8UC3);

    std::stringstream txt;
    txt << n;
    cv::rectangle(msg->value, cv::Rect(0,0, msg->value.cols, msg->value.rows), cv::Scalar::all(0), CV_FILLED);
    cv::putText(msg->value, txt.str(), cv::Point(200, 200), CV_FONT_HERSHEY_PLAIN, 5.0, cv::Scalar::all(255), 2, CV_AA);

    output_->publish(msg);

    ++n;
}


void NumberGenerator::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::AnyMessage>("Trigger");
    output_ = addOutput<connection_types::CvMatMessage>("Image");
}
