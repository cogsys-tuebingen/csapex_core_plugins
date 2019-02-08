/// HEADER
#include "and.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::AND, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;
using namespace csapex::connection_types;

AND::AND()
{
}

void AND::setup(NodeModifier& node_modifier)
{
    in_a = node_modifier.addInput<bool>("A");
    in_b = node_modifier.addInput<bool>("B");

    out = node_modifier.addOutput<bool>("A AND B");
}

void AND::process()
{
    bool a = msg::getValue<bool>(in_a);
    bool b = msg::getValue<bool>(in_b);

    msg::publish(out, (a && b));
}
