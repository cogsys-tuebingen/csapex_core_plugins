/// HEADER
#include "transform_combiner.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformCombiner, csapex::Node)

using namespace csapex;

TransformCombiner::TransformCombiner()
{
}

void TransformCombiner::process()
{
    connection_types::TransformMessage::Ptr a = input_a_->getMessage<connection_types::TransformMessage>();
    connection_types::TransformMessage::Ptr b = input_b_->getMessage<connection_types::TransformMessage>();

    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage(*a));
    msg->value = a->value * b->value;
    msg->frame_id = a->frame_id;
    msg->child_frame = b->child_frame;
    output_->publish(msg);
}


void TransformCombiner::setup()
{
    input_a_ = modifier_->addInput<connection_types::TransformMessage>("A");
    input_b_ = modifier_->addInput<connection_types::TransformMessage>("B");

    output_ = modifier_->addOutput<connection_types::TransformMessage>("A*B");
}
