/// HEADER
#include "set_color.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::SetColor, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SetColor::SetColor()
{
}

void SetColor::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(input_);
    CvMatMessage::ConstPtr in_mask = msg::getMessage<CvMatMessage>(input_mask_);
    CvMatMessage::Ptr out(new CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

    if (in->value.type() != CV_8UC3)
        throw std::runtime_error("Need 3 channel color image!");
    if (in_mask->value.type() != CV_8UC1)
        throw std::runtime_error("Need 1 channel uchar mask!");

    const std::vector<int>& c = readParameter<std::vector<int>>("color");
    cv::Vec3b color(c[2], c[1], c[0]);

    out->value = in->value.clone();
    out->value.setTo(color, in_mask->value);

    msg::publish(output_, out);
}

void SetColor::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Image");
    input_mask_ = node_modifier.addInput<CvMatMessage>("Mask");
    output_ = node_modifier.addOutput<CvMatMessage>("Colored");
}

void SetColor::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::factory::declareColorParameter("color", csapex::param::ParameterDescription("Color to set."), 0, 0, 0));
}
