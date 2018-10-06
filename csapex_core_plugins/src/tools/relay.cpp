/// HEADER
#include "relay.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Relay, csapex::Node)

using namespace csapex;

Relay::Relay() : input_(nullptr), output_(nullptr)
{
}

void Relay::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
    output_ = node_modifier.addOutput<connection_types::AnyMessage>("Same as input");
}

void Relay::process()
{
    TokenData::ConstPtr msg = msg::getMessage<TokenData>(input_);

    msg::publish(output_, msg);
}
