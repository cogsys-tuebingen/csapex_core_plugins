/// HEADER
#include "export_cout.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/message_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ExportCout, csapex::Node)

using namespace csapex;

ExportCout::ExportCout()
    : connector_(NULL)
{
}

void ExportCout::setup()
{
    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}

void ExportCout::process()
{
    ConnectionType::Ptr msg = connector_->getMessage<ConnectionType>();

    ainfo << "writing to cout: ";

    YAML::Node node = MessageFactory::serializeMessage(msg);
    ainfo << node;
    ainfo << std::endl;

    StreamInterceptor::instance().cout << node << "\n---" << std::endl;
}
