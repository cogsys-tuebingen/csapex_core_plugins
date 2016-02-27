/// HEADER
#include "text_convert.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::TextConvert, csapex::Node)

using namespace csapex;

TextConvert::TextConvert()
    : input_(nullptr), output_(nullptr)
{
}

void TextConvert::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<std::string>("Text");
    output_ = node_modifier.addOutput<int>("Number");
}

void TextConvert::process()
{
    std::string text = msg::getValue<std::string>(input_);

    int result = -1;
    if(!text.empty()) {
        result = std::atoi(text.c_str());
    }
    msg::publish(output_, result);
}
