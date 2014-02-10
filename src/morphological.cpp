/// HEADER
#include "morphological.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/assign/std.hpp>

CSAPEX_REGISTER_CLASS(csapex::Morpholocial, csapex::Node)

using namespace csapex;

Morpholocial::Morpholocial()
{
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declareRange<int>("size", 1, 20, 2, 1));
    addParameter(param::ParameterFactory::declareRange<int>("iterations", 0, 10, 1, 1));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("MORPH_OPEN", (int) cv::MORPH_OPEN)
            ("MORPH_CLOSE", (int) cv::MORPH_CLOSE)
            ("MORPH_GRADIENT", (int) cv::MORPH_GRADIENT)
            ("MORPH_TOPHAT", (int) cv::MORPH_TOPHAT)
            ("MORPH_BLACKHAT", (int) cv::MORPH_BLACKHAT);

    addParameter(param::ParameterFactory::declareParameterSet<int>("type", types));

    std::map<std::string, int> elem = boost::assign::map_list_of
            ("MORPH_RECT", (int) cv::MORPH_RECT)
            ("MORPH_CROSS", (int) cv::MORPH_CROSS)
            ("MORPH_ELLIPSE", (int) cv::MORPH_ELLIPSE);
    addParameter(param::ParameterFactory::declareParameterSet<int>("elem", elem));
}

void Morpholocial::allConnectorsArrived()
{
    connection_types::CvMatMessage::Ptr a = input_->getMessage<connection_types::CvMatMessage>();

    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(a->getEncoding()));
    int op = param<int>("type");
    int morph_elem = param<int>("elem");
    int morph_size = param<int>("size");
    cv::Mat kernel = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size+1, 2*morph_size+1), cv::Point(morph_size, morph_size));
    cv::Point anchor(-1,-1);
    int iterations = param<int>("iterations");
    int border_type = cv::BORDER_CONSTANT;
    const cv::Scalar& border_value = cv::morphologyDefaultBorderValue();

    if(!a->value.empty()) {
        cv::morphologyEx(a->value, msg->value, op, kernel, anchor, iterations, border_type, border_value);
    }

    output_->publish(msg);
}


void Morpholocial::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::CvMatMessage>("A");

    output_ = addOutput<connection_types::CvMatMessage>("morph(A)");
}
