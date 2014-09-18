/// HEADER
#include "text_display.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/message_factory.h>

CSAPEX_REGISTER_CLASS(csapex::TextDisplay, csapex::Node)

using namespace csapex;

TextDisplay::TextDisplay()
    : connector_(NULL)
{
}

void TextDisplay::setup()
{
    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}
void TextDisplay::process()
{
    connection_types::Message::Ptr msg = connector_->getMessage<connection_types::Message>();

    std::stringstream ss;
    YAML::Node node = MessageFactory::serializeMessage(*msg);

    convert(ss, node);

    display_request(ss.str());
}

void TextDisplay::convert(std::stringstream &ss, const YAML::Node &node)
{
    if(node.IsMap()) {
        std::vector<std::string> keys;
        for(YAML::Node::const_iterator it = node.begin(); it != node.end(); ++it) {
            keys.push_back(it->first.as<std::string>());
        }

        std::sort(keys.begin(), keys.end());

        for(std::vector<std::string>::iterator key = keys.begin(); key != keys.end(); ++key) {
            ss << *key << ":\t";
            convert(ss, node[*key]);
            ss << "<br />";
        }

    } else {
        ss << node;
    }
}

