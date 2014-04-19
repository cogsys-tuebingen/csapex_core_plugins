/// HEADER
#include "text_input.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
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
    connector_ = addOutput<connection_types::DirectMessage<std::string> >("Text");
}

void TextInput::publish()
{
    connection_types::DirectMessage<std::string>::Ptr msg(new connection_types::DirectMessage<std::string>);
    msg->value = param<std::string>("text");
    connector_->publish(msg);
}
