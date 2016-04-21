/// HEADER
#include "relay.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/token.h>
#include <csapex/msg/message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>

CSAPEX_REGISTER_CLASS(csapex::Relay, csapex::Node)

using namespace csapex;

Relay::Relay()
    : input_(nullptr), output_(nullptr)
{
}

void Relay::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
    output_ = node_modifier.addOutput<connection_types::AnyMessage>("Same as input");
}

void Relay::process()
{
    Token::ConstPtr msg = msg::getMessage<Token>(input_);

    msg::publish(output_, msg);
}

