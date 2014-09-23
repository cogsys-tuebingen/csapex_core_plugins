/// HEADER
#include "text_convert.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::TextConvert, csapex::Node)

using namespace csapex;

TextConvert::TextConvert()
    : input_(NULL), output_(NULL)
{
}

void TextConvert::setup()
{
    input_  = modifier_->addInput<connection_types::GenericValueMessage<std::string> >("Text");
    output_ = modifier_->addOutput<connection_types::GenericValueMessage<int> >("Number");
}

void TextConvert::process()
{
    connection_types::GenericValueMessage<std::string>::Ptr text_msg = input_->getMessage<connection_types::GenericValueMessage<std::string> >();
    connection_types::GenericValueMessage<int>::Ptr int_msg(new connection_types::GenericValueMessage<int>);

    if(!text_msg->value.empty()) {
        int_msg->value = std::atoi(text_msg->value.c_str());
    } /*else {
        std::string text("empty");
        int_msg->value = text;
    }*/
    output_->publish(int_msg);
}
