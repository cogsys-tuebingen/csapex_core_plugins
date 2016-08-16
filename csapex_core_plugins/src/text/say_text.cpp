/// HEADER
#include "say_text.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/param/parameter_factory.h>

CSAPEX_REGISTER_CLASS(csapex::SayText, csapex::Node)

using namespace csapex;

SayText::SayText()
    : connector_(nullptr)
{
}

void SayText::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addInput<std::string>("Text");
}

void SayText::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareBool("repeat", false), repeat_);
}

void SayText::process()
{
    std::string msg = msg::getValue<std::string>(connector_);

    if(msg != last_ || repeat_) {
        last_ = msg;

        if(!msg.empty()) {
            std::stringstream cmd;
            cmd << "espeak \"" << msg << "\" 2> /dev/null 1> /dev/null";
            if(system(cmd.str().c_str())) {
                aerr << "command failed: " << cmd.str() << std::endl;
            }
        }
    }
}
