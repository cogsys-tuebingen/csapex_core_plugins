/// HEADER
#include "say_text.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::SayText, csapex::Node)

using namespace csapex;

SayText::SayText()
    : connector_(NULL)
{
}

void SayText::setup()
{
    connector_ = modifier_->addInput<connection_types::GenericValueMessage<std::string> >("Text");
}

void SayText::process()
{
    connection_types::GenericValueMessage<std::string>::Ptr msg = connector_->getMessage<connection_types::GenericValueMessage<std::string> >();

    if(!msg->value.empty()) {
        std::stringstream cmd;
        cmd << "espeak \"" << msg->value << "\" 2> /dev/null 1> /dev/null &";
        system(cmd.str().c_str());
    }
}
