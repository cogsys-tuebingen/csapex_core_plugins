/// HEADER
#include "indicator.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::Indicator, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;
using namespace csapex::connection_types;

Indicator::Indicator()
{
}

void Indicator::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareBool("signal", true));
}

void Indicator::setup(NodeModifier& node_modifier)
{
    in = node_modifier.addInput<bool>("Signal");
}

void Indicator::process()
{
    bool a = msg::getValue<bool>(in);
    getParameter("signal")->set<bool>(a);
}
