/// HEADER
#include "import_cin.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/message_factory.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/model/message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ImportCin, csapex::Node)

using namespace csapex;

ImportCin::ImportCin()
    : connector_(NULL)
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
        YAML::Node doc;

        YAML::Parser parser(docstream);
        while(getNextDocument(parser, doc)) {
            ConnectionType::Ptr msg;
            try {
                msg = MessageFactory::readYaml(doc);

            } catch(const MessageFactory::DeserializationError& e) {
                ainfo << "could not deserialize message: \n";
                YAML::Emitter e;
                e << doc;
                ainfo << e.c_str() << std::endl;
                continue;
            }

            connector_->setType(msg->toType());
            connector_->publish(msg);
        }
    } catch(YAML::ParserException& e) {
        ainfo << "YAML::ParserException: " << e.what() << "\n";
    }

}
