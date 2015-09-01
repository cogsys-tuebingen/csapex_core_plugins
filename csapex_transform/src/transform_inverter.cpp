/// HEADER
#include "transform_inverter.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformInverter, csapex::Node)

using namespace csapex;

TransformInverter::TransformInverter()
{
}

void TransformInverter::process()
{
    connection_types::TransformMessage::ConstPtr trafo = msg::getMessage<connection_types::TransformMessage>(input_);

    // inverter flips the frames
    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage(trafo->child_frame, trafo->frame_id));
    msg->value = trafo->value.inverse();

    msg::publish(output_, msg);
}


void TransformInverter::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::TransformMessage>("T");

    output_ = node_modifier.addOutput<connection_types::TransformMessage>("T^-1");
}
