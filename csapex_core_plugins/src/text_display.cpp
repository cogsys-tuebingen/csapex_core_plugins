/// HEADER
#include "text_display.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/message_factory.h>
#include <csapex/utility/timer.h>

CSAPEX_REGISTER_CLASS(csapex::TextDisplay, csapex::Node)

using namespace csapex;

TextDisplay::TextDisplay()
    : connector_(nullptr)
{
}

void TextDisplay::setup()
{
    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}
void TextDisplay::process()
{
    connection_types::Message::ConstPtr msg = connector_->getMessage<connection_types::Message>();

    YAML::Node node;
    {
        INTERLUDE("serialize");
        node = MessageFactory::serializeMessage(*msg);
    }

    std::stringstream ss;
    {
        INTERLUDE("convert");
        convert(ss, node, "");
    }

    display_request(ss.str());
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
        for(std::size_t i = 0, n = node.size(); i < n; ++i) {
//            ss << prefix + "-";
            convert(ss, node[i], prefix + "|");
            ss << "\n";
        }

    } else {
        ss << prefix << node.as<std::string>().substr(0, 1000);
    }
}

