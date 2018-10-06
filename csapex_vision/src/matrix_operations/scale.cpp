/// HEADER
#include "scale.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::Scale, csapex::Node)

using namespace csapex::connection_types;
using namespace csapex;
using namespace csapex;

Scale::Scale()
{
}

void Scale::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

    if (!in->value.empty()) {
        cv::resize(in->value, out->value, cv::Size(), scales_[0] / 100.0, scales_[1] / 100.0, mode_);
    } else {
        throw std::runtime_error("Cannot scale empty images!");
    }

    msg::publish(output_, out);
}

void Scale::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("scale");
}

void Scale::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("percent x", 1.0, 400.0, 100.0, 1.0), scales_[0]);
    parameters.addParameter(csapex::param::factory::declareRange("percent y", 1.0, 400.0, 100.0, 1.0), scales_[1]);
    std::map<std::string, int> modes = {
        { "nearest", (int)cv::INTER_NEAREST }, { "linear", (int)cv::INTER_LINEAR }, { "area", (int)cv::INTER_AREA }, { "cubic", (int)cv::INTER_CUBIC }, { "lanczos4", (int)cv::INTER_LANCZOS4 }
    };
    parameters.addParameter(csapex::param::factory::declareParameterSet("mode", modes, (int)cv::INTER_NEAREST), mode_);
}
