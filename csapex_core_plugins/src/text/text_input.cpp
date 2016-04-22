/// HEADER
#include "text_input.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/signal/event.h>

CSAPEX_REGISTER_CLASS(csapex::TextInput, csapex::Node)

using namespace csapex;

TextInput::TextInput()
{
}

void TextInput::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareText("text", ""), [this](param::Parameter* p){
        std::string txt = p->as<std::string>();
        if(txt != text_) {
            text_ = txt;

            auto token = std::make_shared<connection_types::GenericValueMessage<std::string>>();
            token->value = text_;
            event_->triggerWith(token);
        }
    });
}

void TextInput::process()
{
}

void TextInput::tick()
{
    publish();
}

void TextInput::setup(NodeModifier& node_modifier)
{
    output_ = node_modifier.addOutput<std::string>("Text");
    event_ = node_modifier.addEvent("text changed");
}

void TextInput::publish()
{
    msg::publish(output_, text_);
}
