/// HEADER
#include "import_cin.h"

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>

CSAPEX_REGISTER_CLASS(csapex::ImportCin, csapex::Node)

using namespace csapex;

ImportCin::ImportCin()
    : connector_(nullptr)
{
}

void ImportCin::process()
{

}

void ImportCin::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addOutput<connection_types::AnyMessage>("Anything");
}

void ImportCin::tick()
{
    readCin();

    publishNextMessage();
}

void ImportCin::publishNextMessage()
{
    if(!message_buffer_.empty()) {
        auto msg = message_buffer_.front();
        message_buffer_.pop_front();

        msg::publish(connector_, msg);
    }
}

void ImportCin::readCin()
{
    std::string in;
    in = StreamInterceptor::instance().getCin();
    buffer << in;

    readMessages();
}


void ImportCin::readMessages()
{
    std::string buffered = buffer.str();
    int pos = buffered.find("---");

    while(pos >= 0) {
        std::string message = buffered.substr(0, pos);
        buffered = buffered.substr(pos+3);

        try {
            YAML::Node doc = YAML::Load(message);

            try {
                ConnectionType::Ptr msg = MessageSerializer::readYaml(doc);
                message_buffer_.push_back(msg);

            } catch(const MessageSerializer::DeserializationError& e) {
                ainfo << "could not deserialize message: \n";
                ainfo << doc << std::endl;
                ainfo << e.what();
            }


        } catch(YAML::ParserException& e) {
            ainfo << "YAML::ParserException: " << e.what() << ", node was: " << message << "\n";
        }

        pos = buffered.find("---");
    }

    buffer.str(std::string());
    buffer << buffered;
}
