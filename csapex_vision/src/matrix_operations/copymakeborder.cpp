/// HEADER
#include "copymakeborder.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::CopyMakeBorder, csapex::Node)

using namespace csapex;
using namespace connection_types;

CopyMakeBorder::CopyMakeBorder()
{
}

void CopyMakeBorder::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(input_);
    CvMatMessage::Ptr out(new CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

    int type = readParameter<int>("type");
    int left = readParameter<int>("left");
    int top = readParameter<int>("top");
    int right = readParameter<int>("right");
    int bottom = readParameter<int>("bottom");

    cv::copyMakeBorder(in->value, out->value, top, bottom, left, right, type);

    msg::publish(output_, out);
}

void CopyMakeBorder::setupParameters(Parameterizable& parameters)
{
    /*
     Various border types, image boundaries are denoted with '|'

     * BORDER_REPLICATE:     aaaaaa|abcdefgh|hhhhhhh
     * BORDER_REFLECT:       fedcba|abcdefgh|hgfedcb
     * BORDER_REFLECT_101:   gfedcb|abcdefgh|gfedcba
     * BORDER_WRAP:          cdefgh|abcdefgh|abcdefg
     * BORDER_CONSTANT:      iiiiii|abcdefgh|iiiiiii  with some specified 'i'
     */

    std::map<std::string, int> types = { { "REPLICATE", cv::BORDER_REPLICATE }, { "REFLECT", cv::BORDER_REFLECT }, { "REFLECT_101", cv::BORDER_REFLECT_101 }, { "WRAP", cv::BORDER_WRAP } };

    parameters.addParameter(csapex::param::factory::declareParameterSet("type", types, (int)cv::BORDER_REPLICATE));
    parameters.addParameter(csapex::param::factory::declareRange("left", 1, 1000, 1, 1));
    parameters.addParameter(csapex::param::factory::declareRange("top", 1, 1000, 1, 1));
    parameters.addParameter(csapex::param::factory::declareRange("right", 1, 1000, 1, 1));
    parameters.addParameter(csapex::param::factory::declareRange("bottom", 1, 1000, 1, 1));
}

void CopyMakeBorder::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Image");
    output_ = node_modifier.addOutput<CvMatMessage>("Expanded Image");
}
