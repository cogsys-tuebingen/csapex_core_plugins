/// HEADER
#include "times.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <opencv2/core.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::Times, csapex::Node)

Times::Times()
{
}

void Times::process()
{
    CvMatMessage::ConstPtr in_a = msg::getMessage<connection_types::CvMatMessage>(input_a_);
    CvMatMessage::ConstPtr in_b = msg::getMessage<connection_types::CvMatMessage>(input_b_);

    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::unknown, in_a->frame_id, in_a->stamp_micro_seconds));

    cv::multiply(in_a->value, in_b->value, out->value, scale_, dtype_);

    msg::publish(output_, out);
}

void Times::setup(NodeModifier& node_modifier)
{
    input_a_ = node_modifier.addInput<CvMatMessage>("a");
    input_b_ = node_modifier.addInput<CvMatMessage>("b");
    output_ = node_modifier.addOutput<CvMatMessage>("a x b");
}

void Times::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("scale", -255.0, 255.0, 1.0, 0.1),
                            scale_);

    std::map<std::string, int> types =
    {
        {"CV_8U", CV_8U},
        {"CV_8S", CV_8S},
        {"CV_16U", CV_16U},
        {"CV_16S", CV_16S},
        {"CV_32S", CV_32S},
        {"CV_32F", CV_32F},
        {"CV_64F", CV_64F}
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("mode", types, CV_32F),
                            dtype_);
}

