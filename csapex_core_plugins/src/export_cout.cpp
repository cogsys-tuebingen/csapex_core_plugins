/// HEADER
#include "export_cout.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/serialization/message_serializer.h>
#include <csapex/msg/any_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ExportCout, csapex::Node)

using namespace csapex;

ExportCout::ExportCout()
    : connector_(nullptr)
{
}

void ExportCout::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
}

void ExportCout::process()
{
    ConnectionType::ConstPtr msg = msg::getMessage<ConnectionType>(connector_);

//    ainfo << "writing to cout: ";

    YAML::Node node = MessageSerializer::serializeMessage(*msg);
//    ainfo << node;
//    ainfo << std::endl;

    StreamInterceptor::instance().cout << node << "\n---" << std::endl;
}
