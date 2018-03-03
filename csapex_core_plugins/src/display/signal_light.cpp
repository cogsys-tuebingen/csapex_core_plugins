/// HEADER
#include "signal_light.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_handle.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/any_message.h>
#include <csapex/io/raw_message.h>
#include <csapex/serialization/serialization_buffer.h>

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
    slot_red_ = node_modifier.addSlot<connection_types::AnyMessage>("Red", [this](const TokenConstPtr& token){
        setParameter("state", 2);
    });
    slot_yellow_ = node_modifier.addSlot<connection_types::AnyMessage>("Yellow", [this](const TokenConstPtr& token){
        setParameter("state", 1);
    });
    slot_green_ = node_modifier.addSlot<connection_types::AnyMessage>("Green", [this](const TokenConstPtr& token){
        setParameter("state", 0);
    });
}

void SignalLight::process()
{
}

void SignalLight::display(int state)
{
    std::shared_ptr<RawMessage> msg = std::make_shared<RawMessage>(std::vector<uint8_t>{static_cast<uint8_t>(state)},
                                                                   getUUID().getAbsoluteUUID());
    node_handle_->raw_data_connection(msg);
}
