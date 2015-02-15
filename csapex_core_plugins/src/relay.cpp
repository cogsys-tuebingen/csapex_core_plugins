/// HEADER
#include "relay.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Relay, csapex::Node)

using namespace csapex;

Relay::Relay()
    : input_(nullptr), output_(nullptr)
{
}

void Relay::setup()
{
    input_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
    output_ = modifier_->addOutput<connection_types::AnyMessage>("Same as input");
}

void Relay::process()
{
    ConnectionType::ConstPtr msg = msg::getMessage<ConnectionType>(input_);

    msg::publish(output_, msg);
}

