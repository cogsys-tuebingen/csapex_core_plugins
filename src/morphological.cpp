/// HEADER
#include "morphological.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Morpholocial, csapex::Node)

using namespace csapex;

Morpholocial::Morpholocial()
{
    addTag(Tag::get("Vision"));

    addParameter(param::Parameter::declare<int>("size", 1, 20, 2, 1));
    addParameter(param::Parameter::declare<int>("iterations", 1, 10, 1, 1));
}

void Morpholocial::allConnectorsArrived()
{
    connection_types::CvMatMessage::Ptr a = input_->getMessage<connection_types::CvMatMessage>();

    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage);
    int op = cv::MORPH_OPEN;//, MORPH_CLOSE, MORPH_GRADIENT, MORPH_TOPHAT, MORPH_BLACKHAT;
    int morph_elem = cv::MORPH_RECT;
    int morph_size = param<int>("size");
    cv::Mat kernel = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size+1, 2*morph_size+1), cv::Point(morph_size, morph_size));
    cv::Point anchor(-1,-1);
    int iterations = param<int>("iterations");
    int border_type = cv::BORDER_CONSTANT;
    const cv::Scalar& border_value = cv::morphologyDefaultBorderValue();

    cv::morphologyEx(a->value, msg->value, op, kernel, anchor, iterations, border_type, border_value);

    output_->publish(msg);
}


void Morpholocial::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::CvMatMessage>("A");

    output_ = addOutput<connection_types::CvMatMessage>("morph(A)");
}
