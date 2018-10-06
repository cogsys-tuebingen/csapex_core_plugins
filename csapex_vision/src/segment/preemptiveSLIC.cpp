/// HEADER
#include "preemptiveSLIC.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <cslibs_vision/utils/preemptiveSLIC.h>

CSAPEX_REGISTER_CLASS(csapex::PreemptiveSLIC, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

PreemptiveSLIC::PreemptiveSLIC()
{
}

void PreemptiveSLIC::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::unknown, in->frame_id, in->stamp_micro_seconds));

    if (in->value.type() != CV_8UC3)
        throw std::runtime_error("Need a 3 channel bgr image!");

    int n = readParameter<int>("super pixels");
    double c = readParameter<double>("compactness");

    cslibs_vision::PreemptiveSLIC ps;
    cv::Mat seeds;
    ps.preemptiveSLIC(in->value, n, c, out->value, seeds);

    msg::publish(output_, out);
}

void PreemptiveSLIC::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("image");
    output_ = node_modifier.addOutput<CvMatMessage>("clusters");
}

void PreemptiveSLIC::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::factory::declareRange("super pixels", 10, 2000, 50, 1));
    addParameter(csapex::param::factory::declareRange("compactness", 0.0, 200.0, 0.0, 0.1));
}
