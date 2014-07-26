/// HEADER
#include "transform_inverter.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformInverter, csapex::Node)

using namespace csapex;

TransformInverter::TransformInverter()
{
}

void TransformInverter::process()
{
    connection_types::TransformMessage::Ptr trafo = input_->getMessage<connection_types::TransformMessage>();
    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = trafo->value.inverse();
    output_->publish(msg);
}


void TransformInverter::setup()
{
    input_ = modifier_->addInput<connection_types::TransformMessage>("T");

    output_ = modifier_->addOutput<connection_types::TransformMessage>("T^-1");
}
