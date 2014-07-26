/// HEADER
#include "relay.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Relay, csapex::Node)

using namespace csapex;

Relay::Relay()
    : input_(NULL), output_(NULL)
{
}

void Relay::setup()
{
    input_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
    output_ = modifier_->addOutput<connection_types::AnyMessage>("Same as input");
}

void Relay::process()
{
    ConnectionType::Ptr msg = input_->getMessage<ConnectionType>();

    output_->setType(input_->getType());
    output_->publish(msg);
}

