/// HEADER
#include "toggle.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::Toggle, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;

Toggle::Toggle()
{
}

void Toggle::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareBool("true", true), std::bind(&Toggle::setSignal, this));
}

void Toggle::setup(NodeModifier& node_modifier)
{
    out_ = node_modifier.addOutput<bool>("Signal");
    event_ = node_modifier.addEvent("Changed");
}

void Toggle::process()
{
    msg::publish(out_, signal_);
}

void Toggle::setSignal()
{
    auto s = readParameter<bool>("true");
    if (signal_ != s) {
        signal_ = s;
        msg::trigger(event_);
    }
}
