/// HEADER
#include "export_ros.h"

/// COMPONENT
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

/// SYSTEM
#include <QPushButton>

CSAPEX_REGISTER_CLASS(csapex::ExportRos, csapex::Node)

using namespace csapex;

ExportRos::ExportRos()
    : connector_(NULL), create_pub(false)
{
}

void ExportRos::setupParameters()
{
    addParameter(param::ParameterFactory::declareText("topic", "export"),
                 boost::bind(&ExportRos::updateTopic, this));
}

void ExportRos::setup()
{
    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}

void ExportRos::setupROS()
{

}

void ExportRos::processROS()
{
    if(topic_.empty()) {
        return;
    }

    ConnectionType::Ptr msg = connector_->getMessage<ConnectionType>();

    connection_types::VectorMessage::Ptr vector = boost::dynamic_pointer_cast<connection_types::VectorMessage>(msg);

    ConnectionType::Ptr type;
    if(vector) {
        type = vector->getSubType();
    } else {
        type = msg->toType();
    }

    if(create_pub) {
        pub = RosMessageConversion::instance().advertise(type, topic_, 1, true);
        create_pub = false;

        connector_->setLabel(pub.getTopic());
        connector_->setType(type);
    }

    if(vector) {
        for(std::vector<ConnectionType::Ptr>::iterator it = vector->value.begin();
            it != vector->value.end();
            ++it) {
            RosMessageConversion::instance().publish(pub, *it);
        }
    } else {
        RosMessageConversion::instance().publish(pub, msg);
    }
}

void ExportRos::updateTopic()
{
    topic_ = readParameter<std::string>("topic");
    create_pub = true;
}
