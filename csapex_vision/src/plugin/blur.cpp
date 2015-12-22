/// HEADER
#include "blur.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::BoxBlur, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

BoxBlur::BoxBlur()
{
}

void BoxBlur::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    int kernel = readParameter<int>("kernel");

    cv::blur(in->value,out->value, cv::Size(kernel, kernel));

    msg::publish(output_, out);
}

void BoxBlur::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Unblurred");
    output_ = node_modifier.addOutput<CvMatMessage>("Blurred");
}

void BoxBlur::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("kernel", 1, 255, 1, 2));
}
