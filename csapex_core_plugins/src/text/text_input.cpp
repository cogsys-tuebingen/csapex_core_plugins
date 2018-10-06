/// HEADER
#include "text_input.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/model/token.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TextInput, csapex::Node)

using namespace csapex;

TextInput::TextInput()
{
}

void TextInput::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareText("text", ""), [this](param::Parameter* p) {
        std::string txt = p->as<std::string>();
        if (txt != text_) {
            text_ = txt;

            auto text_message = std::make_shared<connection_types::GenericValueMessage<std::string>>();
            text_message->value = text_;
            event_->triggerWith(std::make_shared<Token>(text_message));
        }
    });
}

void TextInput::process()
{
    msg::publish(output_, text_);
}

void TextInput::setup(NodeModifier& node_modifier)
{
    output_ = node_modifier.addOutput<std::string>("Text");
    event_ = node_modifier.addEvent("text changed");
}
