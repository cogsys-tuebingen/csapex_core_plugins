/// HEADER
#include "indicator.h"

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

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
    in = modifier_->addInput<GenericValueMessage<bool> >("Signal");
}

void Indicator::process()
{
    GenericValueMessage<bool>::Ptr a = in->getMessage<GenericValueMessage<bool> >();
    getParameter("signal")->set<bool>(a->value);
}
