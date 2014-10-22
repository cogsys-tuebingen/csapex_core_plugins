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
    input_  = modifier_->addInput<std::string>("Text");
    output_ = modifier_->addOutput<int>("Number");
}

void TextConvert::process()
{
    std::string text = input_->getValue<std::string>();

    int result = -1;
    if(!text.empty()) {
        result = std::atoi(text.c_str());
    }
    output_->publish(result);
}
