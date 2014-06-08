/// HEADER
#include "export_ros.h"

/// COMPONENT
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/model/message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <QPushButton>

CSAPEX_REGISTER_CLASS(csapex::ExportRos, csapex::Node)

using namespace csapex;

ExportRos::ExportRos()
    : connector_(NULL), create_pub(false)
{
    addTag(Tag::get("RosIO"));
    addTag(Tag::get("General"));
    addTag(Tag::get("Output"));

    addParameter(param::ParameterFactory::declareText("topic", "export"),
                 boost::bind(&ExportRos::updateTopic, this));
}

QIcon ExportRos::getIcon() const
{
    return QIcon(":/terminal.png");
}

void ExportRos::setup()
{

    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}

void ExportRos::process()
{
    if(topic_.empty()) {
        return;
    }

    ConnectionType::Ptr msg = connector_->getMessage<ConnectionType>();

    if(create_pub) {
        pub = RosMessageConversion::instance().advertise(msg->toType(), topic_, 1, true);
        create_pub = false;

        connector_->setLabel(pub.getTopic());
        connector_->setType(msg);
    }

    if(create_pub) {
        setError(true, "Publisher is not valid", EL_WARNING);
        return;
    }

    RosMessageConversion::instance().publish(pub, msg);
}

void ExportRos::updateTopic()
{
    topic_ = param<std::string>("topic");
    create_pub = true;
}
