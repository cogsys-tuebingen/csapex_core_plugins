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
    in_a = modifier_->addInput<bool>("A");
    in_b = modifier_->addInput<bool>("B");

    out = modifier_->addOutput<bool>("A nand B");
}

void NAND::process()
{
    bool a = in_a->getValue<bool>();
    bool b = in_b->getValue<bool>();

    out->publish(!(a && b));
}
