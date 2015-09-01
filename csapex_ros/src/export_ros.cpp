/// HEADER
#include "export_ros.h"

/// COMPONENT
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>

/// PROJECT
#include <csapex/msg/io.h>
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
    : connector_(nullptr), create_pub(false)
{
}

void ExportRos::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareText("topic", "export"),
                 std::bind(&ExportRos::updateTopic, this));
}

void ExportRos::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
}

void ExportRos::setupROS()
{

}

void ExportRos::processROS()
{
    if(topic_.empty()) {
        return;
    }

    ConnectionType::ConstPtr msg = msg::getMessage<ConnectionType>(connector_);

    connection_types::VectorMessage::ConstPtr vector = std::dynamic_pointer_cast<connection_types::VectorMessage const>(msg);

    ConnectionType::ConstPtr type;
    if(vector) {
        type = vector->getSubType();
    } else {
        type = msg;//->toType();
    }

    if(create_pub) {
        pub = RosMessageConversion::instance().advertise(type, topic_, 1, true);
        create_pub = false;

        msg::setLabel(connector_, pub.getTopic());
    }

    if(vector) {
        for(auto it = vector->value.begin();
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
