/// HEADER
#include "text_input.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::TextInput, csapex::Node)

using namespace csapex;

TextInput::TextInput()
{
}

void TextInput::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareText("text", ""));
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
    connector_ = node_modifier.addOutput<std::string>("Text");
}

void TextInput::publish()
{
    msg::publish(connector_, readParameter<std::string>("text"));
}
