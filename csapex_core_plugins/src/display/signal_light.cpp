/// HEADER
#include "signal_light.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/any_message.h>

CSAPEX_REGISTER_CLASS(csapex::SignalLight, csapex::Node)

using namespace csapex;

SignalLight::SignalLight()
{
}

void SignalLight::setupParameters(Parameterizable &parameters)
{
    std::map<std::string, int> states = {
        {"green", 0},
        {"yellow", 1},
        {"red", 2}
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("state", states, 0),
                            [this](param::Parameter* p) {
        display(p->as<int>());
    });
}

void SignalLight::setup(NodeModifier& node_modifier)
{
    slot_red_ = node_modifier.addTypedSlot<connection_types::AnyMessage>("Red", [this](const TokenConstPtr& token){
        setParameter("state", 2);
    });
    slot_yellow_ = node_modifier.addTypedSlot<connection_types::AnyMessage>("Yellow", [this](const TokenConstPtr& token){
        setParameter("state", 1);
    });
    slot_green_ = node_modifier.addTypedSlot<connection_types::AnyMessage>("Green", [this](const TokenConstPtr& token){
        setParameter("state", 0);
    });
}

void SignalLight::process()
{
}

void SignalLight::display(int state)
{
    display_request(state);
}
