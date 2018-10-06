/// HEADER
#include "resize.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::Resize, csapex::Node)

using namespace csapex::connection_types;
using namespace csapex;
using namespace csapex;

Resize::Resize()
{
}

void Resize::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

    if (!in->value.empty()) {
        cv::resize(in->value, out->value, size_, 0.0, 0.0, mode_);
    } else {
        throw std::runtime_error("Cannot scale empty images!");
    }

    msg::publish(output_, out);
}

void Resize::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("resize");
    update();
}

void Resize::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("size width", 1, 10000, 640, 1), std::bind(&Resize::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("size height", 1, 10000, 480, 1), std::bind(&Resize::update, this));

    std::map<std::string, int> modes = {
        { "nearest", (int)CV_INTER_NN }, { "linear", (int)CV_INTER_LINEAR }, { "area", (int)CV_INTER_AREA }, { "cubic", (int)CV_INTER_CUBIC }, { "lanczos4", (int)CV_INTER_LANCZOS4 }
    };
    parameters.addParameter(csapex::param::factory::declareParameterSet<int>("mode", modes, (int)cv::INTER_NEAREST), std::bind(&Resize::update, this));
}

void Resize::update()
{
    size_.width = readParameter<int>("size width");
    size_.height = readParameter<int>("size height");
    mode_ = readParameter<int>("mode");
}
