/// HEADER
#include "equalize.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <cslibs_vision/utils/histogram.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::Equalize, csapex::Node)

Equalize::Equalize()
{
}

void Equalize::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));
    out->value = in->value.clone();
    cslibs_vision::histogram::full_channel_equalize(out->value, out->value);
    msg::publish(output_, out);
}

void Equalize::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("equalized");
}
