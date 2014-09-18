/// HEADER
#include "text_display.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/message_factory.h>

CSAPEX_REGISTER_CLASS(csapex::TextDisplay, csapex::Node)

using namespace csapex;

TextDisplay::TextDisplay()
    : connector_(NULL)
{
}

void TextDisplay::setup()
{
    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}

void TextDisplay::process()
{
    connection_types::Message::Ptr msg = connector_->getMessage<connection_types::Message>();

    std::stringstream ss;
    ss << MessageFactory::serializeMessage(*msg);

    display_request(ss.str());
}
