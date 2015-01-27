/// HEADER
#include "import_cin.h"

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/message_factory.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/yaml_node_builder.h>

CSAPEX_REGISTER_CLASS(csapex::ImportCin, csapex::Node)

using namespace csapex;

ImportCin::ImportCin()
    : connector_(nullptr)
{
}

void ImportCin::process()
{

}

void ImportCin::setup()
{
    connector_ = modifier_->addOutput<connection_types::AnyMessage>("Anything");
}

void ImportCin::tick()
{
    std::string in;
    in = StreamInterceptor::instance().getCin();
    buffer << in;

    std::string unused = buffer.str();

    int pos = unused.find_first_of("---");
    if(pos == -1 || unused.empty()) {
        return;
    }

    std::stringstream docstream;

    int iter = 0;
    while(pos != -1) {
        std::string sub = unused.substr(0, pos);

        unused = unused.substr(pos+3);

        if(pos != -1) {
            if(!docstream.str().empty()) {
                docstream << "\n---\n";
            }
            docstream << sub;
        }

        pos = unused.find_first_of("---");

        if(iter++ >= 10) {
            ainfo << "break out!" << std::endl;
            break;
        }
    }

    buffer.str(std::string());
    buffer << unused;

    try {
        YAML::Parser parser(docstream);
        YAML::NodeBuilder builder;
        while(parser.HandleNextDocument(builder)) {
            YAML::Node doc = builder.Root();

            ConnectionType::Ptr msg;
            try {
                msg = MessageFactory::readYaml(doc);

            } catch(const MessageFactory::DeserializationError& e) {
                ainfo << "could not deserialize message: \n";
                YAML::Emitter emitter;
                emitter << doc;
                ainfo << emitter.c_str() << std::endl;
                continue;
            }

            connector_->setType(msg->toType());
            connector_->publish(msg);
        }
    } catch(YAML::ParserException& e) {
        ainfo << "YAML::ParserException: " << e.what() << "\n";
    }

}
