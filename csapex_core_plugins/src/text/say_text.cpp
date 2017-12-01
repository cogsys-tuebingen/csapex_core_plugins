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
    : connector_(nullptr),
      speaking_(false)
{
}

void SayText::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addInput<std::string>("Text");
}

void SayText::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareBool("repeat", false), repeat_);
    parameters.addParameter(param::ParameterFactory::declareBool("asynchrounous", true), async_);

    parameters.addParameter(param::ParameterFactory::declareText("language", "en"), language_);
    parameters.addParameter(param::ParameterFactory::declareRange("pitch", 1, 255, 100, 1), pitch_);
    parameters.addParameter(param::ParameterFactory::declareRange("speed", 1, 255, 100, 1), speed_);
}

void SayText::process()
{
    std::string msg = msg::getValue<std::string>(connector_);

    if(msg != last_ || repeat_) {
        if(speaking_) {
            return;
        }

        last_ = msg;

        if(!msg.empty()) {

            auto speak = [this, msg](){
                speaking_ = true;
                std::stringstream cmd;
                cmd << "espeak -v " << language_ << " -p " << pitch_ << " -s " << speed_ << " \"" << msg << "\" 2> /dev/null 1> /dev/null";
                ainfo << cmd.str() << std::endl;
                if(system(cmd.str().c_str())) {
                    aerr << "command failed: " << cmd.str() << std::endl;
                }
                speaking_ = false;
            };

            if(async_) {
                future_ = std::async(std::launch::async, speak);
            } else {
                speak();
            }
        }
    }
}
