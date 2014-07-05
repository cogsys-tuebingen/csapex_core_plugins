/// HEADER
#include "text_input.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <QLineEdit>
#include <QPushButton>
#include <QBoxLayout>

CSAPEX_REGISTER_CLASS(csapex::TextInput, csapex::Node)

using namespace csapex;

TextInput::TextInput()
{
    addTag(Tag::get("Input"));
    addTag(Tag::get("General"));

    addParameter(param::ParameterFactory::declareText("text", ""), boost::bind(&TextInput::publish, this));
}

void TextInput::process()
{

}

QIcon TextInput::getIcon() const
{
    return QIcon(":/pencil.png");
}

void TextInput::setup()
{
    connector_ = modifier_->addOutput<connection_types::GenericValueMessage<std::string> >("Text");
}

void TextInput::publish()
{
    connection_types::GenericValueMessage<std::string>::Ptr msg(new connection_types::GenericValueMessage<std::string>);
    msg->value = param<std::string>("text");
    connector_->publish(msg);
}
