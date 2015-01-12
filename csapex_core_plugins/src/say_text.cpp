/// HEADER
#include "say_text.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::SayText, csapex::Node)

using namespace csapex;

SayText::SayText()
    : connector_(nullptr)
{
}

void SayText::setup()
{
    connector_ = modifier_->addInput<std::string>("Text");
}

void SayText::process()
{
    std::string msg = connector_->getValue<std::string>();

    if(!msg.empty()) {
        std::stringstream cmd;
        cmd << "espeak \"" << msg << "\" 2> /dev/nullptr 1> /dev/nullptr &";
        system(cmd.str().c_str());
    }
}
