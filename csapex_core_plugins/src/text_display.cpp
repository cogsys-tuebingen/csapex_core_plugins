/// HEADER
#include "text_display.h"

/// PROJECT
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TextDisplay, csapex::Node)

using namespace csapex;

TextDisplay::TextDisplay()
    : connector_(NULL)
{
    addTag(Tag::get("Output"));
    addTag(Tag::get("General"));
}

QIcon TextDisplay::getIcon() const
{
    return QIcon(":/pencil.png");
}

void TextDisplay::setup()
{
    connector_ = addInput<connection_types::AnyMessage>("Anything", false, true);
}

void TextDisplay::process()
{
    connection_types::Message::Ptr msg = connector_->getMessage<connection_types::Message>();

    std::stringstream ss;
    msg->write(ss);

    display_request(ss.str());
}
