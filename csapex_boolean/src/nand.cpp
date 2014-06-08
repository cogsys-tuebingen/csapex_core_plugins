/// HEADER
#include "nand.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::NAND, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;
using namespace csapex::connection_types;

NAND::NAND()
{
    addTag(Tag::get("Boolean"));
}

void NAND::setup()
{
    in_a = modifier_->addInput<DirectMessage<bool> >("A");
    in_b = modifier_->addInput<DirectMessage<bool> >("B");

    out = modifier_->addOutput<DirectMessage<bool> >("A nand B");
}

void NAND::process()
{
    DirectMessage<bool>::Ptr a = in_a->getMessage<DirectMessage<bool> >();
    DirectMessage<bool>::Ptr b = in_b->getMessage<DirectMessage<bool> >();

    DirectMessage<bool>::Ptr msg(new DirectMessage<bool>);
    msg->value = !(a->value && b->value);
    out->publish(msg);
}
