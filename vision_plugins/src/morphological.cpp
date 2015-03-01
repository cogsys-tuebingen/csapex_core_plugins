/// HEADER
#include "morphological.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/assign/std.hpp>

CSAPEX_REGISTER_CLASS(csapex::Morpholocial, csapex::Node)

using namespace csapex;

Morpholocial::Morpholocial()
{
}

void Morpholocial::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange<int>("size", 1, 20, 2, 1));
    parameters.addParameter(param::ParameterFactory::declareRange<int>("iterations", 0, 10, 1, 1));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("MORPH_ERODE", (int) cv::MORPH_ERODE)
            ("MORPH_DILATE", (int) cv::MORPH_DILATE)
            ("MORPH_OPEN", (int) cv::MORPH_OPEN)
            ("MORPH_CLOSE", (int) cv::MORPH_CLOSE)
            ("MORPH_GRADIENT", (int) cv::MORPH_GRADIENT)
            ("MORPH_TOPHAT", (int) cv::MORPH_TOPHAT)
            ("MORPH_BLACKHAT", (int) cv::MORPH_BLACKHAT);

    parameters.addParameter(param::ParameterFactory::declareParameterSet<int>("type", types, (int) cv::MORPH_ERODE));

    std::map<std::string, int> elem = boost::assign::map_list_of
            ("MORPH_RECT", (int) cv::MORPH_RECT)
            ("MORPH_CROSS", (int) cv::MORPH_CROSS)
            ("MORPH_ELLIPSE", (int) cv::MORPH_ELLIPSE);
    parameters.addParameter(param::ParameterFactory::declareParameterSet<int>("elem", elem, (int) cv::MORPH_RECT));
}

void Morpholocial::process()
{
    connection_types::CvMatMessage::ConstPtr a = msg::getMessage<connection_types::CvMatMessage>(input_);

    connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage(a->getEncoding(), a->stamp_micro_seconds));
    int op = readParameter<int>("type");
    int morph_elem = readParameter<int>("elem");
    int morph_size = readParameter<int>("size");
    cv::Mat kernel = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size+1, 2*morph_size+1), cv::Point(morph_size, morph_size));
    cv::Point anchor(-1,-1);
    int iterations = readParameter<int>("iterations");
    int border_type = cv::BORDER_CONSTANT;
    const cv::Scalar& border_value = cv::morphologyDefaultBorderValue();

    if(!a->value.empty()) {
        cv::morphologyEx(a->value, msg->value, op, kernel, anchor, iterations, border_type, border_value);
    }

    msg::publish(output_, msg);
}


void Morpholocial::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::CvMatMessage>("original");

    output_ = node_modifier.addOutput<connection_types::CvMatMessage>("morph(original)");
}
