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
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/end_of_sequence_message.h>

CSAPEX_REGISTER_CLASS(csapex::ImportCin, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ImportCin::ImportCin()
    : connector_(nullptr)
{
}
void ImportCin::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addOutput<connection_types::AnyMessage>("Anything");
}

void ImportCin::setupParameters(Parameterizable &params)
{
    params.addParameter(param::ParameterFactory::declareBool("import yaml", true), import_yaml_);
    params.addParameter(param::ParameterFactory::declareBool("latch", false), latch_);
    params.addParameter(param::ParameterFactory::declareBool("signal end", false), signal_end_);
}


void ImportCin::process()
{
    readCin();

    publishNextMessage();
}

void ImportCin::publishNextMessage()
{
    if(!message_buffer_.empty()) {
        auto msg = message_buffer_.front();
        message_buffer_.pop_front();

        last_message_ = msg;

        msg::publish(connector_, msg);

    } else if(latch_ && last_message_) {
        msg::publish(connector_, last_message_);

    } else if(signal_end_) {
        msg::publish(connector_, connection_types::makeEmpty<EndOfSequenceMessage>());
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

        if(import_yaml_) {
            readYAML(message);
        } else {
            auto msg = connection_types::makeEmptyMessage<connection_types::GenericValueMessage<std::string>>();
            msg->value = message;
            message_buffer_.push_back(msg);
        }

        pos = buffered.find("---");
    }

    buffer.str(std::string());
    buffer << buffered;
}



void ImportCin::readYAML(const std::string& message)
{
    try {
        YAML::Node doc = YAML::Load(message);

        try {
            TokenData::Ptr msg = MessageSerializer::readYaml(doc);
            message_buffer_.push_back(msg);

        } catch(const MessageSerializer::DeserializationError& e) {
            ainfo << "could not deserialize message: \n";
            ainfo << doc << std::endl;
            ainfo << e.what();
        }


    } catch(YAML::ParserException& e) {
        ainfo << "YAML::ParserException: " << e.what() << ", node was: " << message << "\n";
    }
}
