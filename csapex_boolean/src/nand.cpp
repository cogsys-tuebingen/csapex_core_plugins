/// HEADER
#include "nand.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::NAND, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;
using namespace csapex::connection_types;

NAND::NAND()
{
}

void NAND::setup(NodeModifier& node_modifier)
{
    in_a = node_modifier.addInput<bool>("A");
    in_b = node_modifier.addInput<bool>("B");

    out = node_modifier.addOutput<bool>("A nand B");
}

void NAND::process()
{
    bool a = msg::getValue<bool>(in_a);
    bool b = msg::getValue<bool>(in_b);

    msg::publish(out, !(a && b));
}
