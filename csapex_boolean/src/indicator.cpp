/// HEADER
#include "indicator.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::boolean::Indicator, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;
using namespace csapex::connection_types;

Indicator::Indicator()
{
    addParameter(param::ParameterFactory::declareBool("signal", true));
}

void Indicator::setup()
{
    in = modifier_->addInput<bool>("Signal");
}

void Indicator::process()
{
    bool a = msg::getValue<bool>(in);
    getParameter("signal")->set<bool>(a);
}
