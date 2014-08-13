/// HEADER
#include "nand.h"

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::NAND, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;
using namespace csapex::connection_types;

NAND::NAND()
{
}

void NAND::setup()
{
    in_a = modifier_->addInput<GenericValueMessage<bool> >("A");
    in_b = modifier_->addInput<GenericValueMessage<bool> >("B");

    out = modifier_->addOutput<GenericValueMessage<bool> >("A nand B");
}

void NAND::process()
{
    GenericValueMessage<bool>::Ptr a = in_a->getMessage<GenericValueMessage<bool> >();
    GenericValueMessage<bool>::Ptr b = in_b->getMessage<GenericValueMessage<bool> >();

    GenericValueMessage<bool>::Ptr msg(new GenericValueMessage<bool>);
    msg->value = !(a->value && b->value);
    out->publish(msg);
}
