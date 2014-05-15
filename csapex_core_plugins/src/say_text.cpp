/// HEADER
#include "say_text.h"

/// PROJECT
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::SayText, csapex::Node)

using namespace csapex;

SayText::SayText()
    : connector_(NULL)
{
    addTag(Tag::get("Output"));
    addTag(Tag::get("General"));
}

QIcon SayText::getIcon() const
{
    return QIcon(":/pencil.png");
}

void SayText::setup()
{
    connector_ = addInput<connection_types::DirectMessage<std::string> >("Text");
}

void SayText::process()
{
    connection_types::DirectMessage<std::string>::Ptr msg = connector_->getMessage<connection_types::DirectMessage<std::string> >();

    if(!msg->value.empty()) {
        std::stringstream cmd;
        cmd << "espeak \"" << msg->value << "\" 2> /dev/null 1> /dev/null &";
        system(cmd.str().c_str());
    }
}
