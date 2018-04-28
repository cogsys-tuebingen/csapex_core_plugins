/// HEADER
#include "text_display.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/profiling/timer.h>
#include <csapex/msg/any_message.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/model/token.h>
#include <csapex/io/raw_message.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/model/node_handle.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <cmath>

CSAPEX_REGISTER_CLASS(csapex::TextDisplay, csapex::Node)

using namespace csapex;

TextDisplay::TextDisplay()
    : input_(nullptr)
{
}

void TextDisplay::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");

    slot_ = node_modifier.addSlot<connection_types::AnyMessage>("Display", [this](const TokenConstPtr& token){
        display(token->getTokenData());
    });
}

void TextDisplay::process()
{
    connection_types::Message::ConstPtr msg = msg::getMessage<connection_types::Message>(input_);

    display(msg);
}

void TextDisplay::display(TokenDataConstPtr msg)
{
    YAML::Node node;
    {
        INTERLUDE("serialize");
        node = MessageSerializer::serializeYamlMessage(*msg);
    }

    std::stringstream ss;
    {
        INTERLUDE("convert");
        convert(ss, node, "");
    }

    SerializationBuffer buffer;
    buffer << ss.str();

    std::shared_ptr<RawMessage> raw_msg = std::make_shared<RawMessage>(buffer, getUUID().getAbsoluteUUID());
    node_handle_->raw_data_connection(raw_msg);
}
void TextDisplay::convert(std::stringstream &ss, const YAML::Node &node, const std::string& prefix)
{
    static const std::string PREFIX = "   ";

    if(node.IsMap()) {
        std::vector<std::string> keys;
        for(YAML::Node::const_iterator it = node.begin(); it != node.end(); ++it) {
            keys.push_back(it->first.as<std::string>());
        }

        std::sort(keys.begin(), keys.end());

        for(std::vector<std::string>::iterator key = keys.begin(); key != keys.end(); ++key) {
            ss << prefix << *key << ": \n";
            convert(ss, node[*key], prefix + PREFIX);
            ss << "\n";
        }

    } else if(node.IsSequence()) {
        std::size_t total = node.size();
        std::size_t max_count = 16;

        for(std::size_t i = 0, n = std::min(max_count, total); i < n; ++i) {
            convert(ss, node[i], prefix + "|");
            ss << "\n";
        }
        if(max_count < total) {
            ss << "...\n";
        }

    } else if(node.Type() != YAML::NodeType::Null) {
        ss << prefix << node.as<std::string>().substr(0, 1000);
    }
}

