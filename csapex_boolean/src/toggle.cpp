/// HEADER
#include "toggle.h"

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::boolean::Toggle, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;

Toggle::Toggle()
{
    addParameter(param::ParameterFactory::declareBool("true", true),
                 boost::bind(&Toggle::setSignal, this));
}

void Toggle::setup()
{
    out = modifier_->addOutput<bool>("Signal");
}

void Toggle::process()
{

}

void Toggle::tick()
{
    out->publish(signal_);
}

void Toggle::setSignal()
{
    signal_ = readParameter<bool>("true");
}
