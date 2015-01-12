/// HEADER
#include "text_input.h"

/// PROJECT
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#include <QLineEdit>
#include <QPushButton>
#include <QBoxLayout>

CSAPEX_REGISTER_CLASS(csapex::TextInput, csapex::Node)

using namespace csapex;

TextInput::TextInput()
{
    addParameter(param::ParameterFactory::declareText("text", ""), boost::bind(&TextInput::publish, this));
}

void TextInput::process()
{

}

void TextInput::setup()
{
    connector_ = modifier_->addOutput<std::string>("Text");
}

void TextInput::publish()
{
    connector_->publish(readParameter<std::string>("text"));
}
