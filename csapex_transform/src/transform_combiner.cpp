/// HEADER
#include "transform_combiner.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex/param/parameter_factory.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformCombiner, csapex::Node)

using namespace csapex;

TransformCombiner::TransformCombiner()
{
}

void TransformCombiner::process()
{
    connection_types::TransformMessage::ConstPtr a = msg::getMessage<connection_types::TransformMessage>(input_a_);
    connection_types::TransformMessage::ConstPtr b = msg::getMessage<connection_types::TransformMessage>(input_b_);

    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage(*a));
    msg->value = a->value * b->value;
    msg->stamp_micro_seconds = which_stamp_ == 0 ? a->stamp_micro_seconds : b->stamp_micro_seconds;
    msg->frame_id = a->frame_id;
    msg->child_frame = b->child_frame;
    msg::publish(output_, msg);
}


void TransformCombiner::setupParameters(Parameterizable& params)
{
    std::map<std::string, int> which = {
        {"FIRST (A)", 0},
        {"SECOND (B)", 1}
    };

    params.addParameter(param::ParameterFactory::declareParameterSet<int>("which_timestamp", which, 0),
                            which_stamp_);
}

void TransformCombiner::setup(NodeModifier& node_modifier)
{
    input_a_ = node_modifier.addInput<connection_types::TransformMessage>("A");
    input_b_ = node_modifier.addInput<connection_types::TransformMessage>("B");

    output_ = node_modifier.addOutput<connection_types::TransformMessage>("A*B");
}
