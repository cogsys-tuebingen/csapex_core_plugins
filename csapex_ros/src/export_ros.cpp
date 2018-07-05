/// HEADER
#include "export_ros.h"

/// COMPONENT
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
#include <QPushButton>

CSAPEX_REGISTER_CLASS(csapex::ExportRos, csapex::Node)

using namespace csapex;

ExportRos::ExportRos()
    : connector_(nullptr), target_type_(-1)
{
}

void ExportRos::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("queue", 1, 32, 1, 1),
                            std::bind(&ExportRos::updateTopic, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareText("topic", "export"),
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

    TokenData::ConstPtr msg = msg::getMessage<TokenData>(connector_);

    connection_types::GenericVectorMessage::ConstPtr vector = std::dynamic_pointer_cast<connection_types::GenericVectorMessage const>(msg);

    TokenData::ConstPtr type;
    if(vector) {
        type = vector->nestedType();
    } else {
        type = msg;//->toType();
    }

    int selected_target_type = -1;
    if(hasParameter("target_type")) {
        selected_target_type = readParameter<int>("target_type");
    }

    if(selected_target_type >= 0 && selected_target_type != target_type_) {
        if(pub) {
            aerr << "Recreate publisher, selected target type has changed from " << target_type_ << " to " << selected_target_type;
            pub.get().shutdown();
            pub.reset();
        }
    }

    target_type_ = selected_target_type;

    if(!pub) {
        RosMessageConversion& ros_conv = RosMessageConversion::instance();

        std::map<std::string, int> possible_conversions = ros_conv.getAvailableRosConversions(type);
        if(possible_conversions.size() == 1) {
            pub = ros_conv.advertise(type, topic_, queue_, true);
            msg::setLabel(connector_, pub.get().getTopic());

        } else {
            if(!hasParameter("target_type")) {
                addTemporaryParameter(param::ParameterFactory::declareParameterSet("target_type", possible_conversions, 0));
                selected_target_type = readParameter<int>("target_type");
            }

            apex_assert(selected_target_type >= 0);

            pub = ros_conv.advertise(type, topic_, queue_, true, selected_target_type);
            msg::setLabel(connector_, pub.get().getTopic());
        }

    }

    if(!pub || !pub.get()) {
        return;
    }

    if(vector) {
        for(std::size_t i = 0, n = vector->nestedValueCount(); i < n; ++i) {
            TokenDataConstPtr msg = vector->nestedValue(i);
            RosMessageConversion::instance().publish(pub.get(), msg);
        }
    } else {
        RosMessageConversion::instance().publish(pub.get(), msg, selected_target_type);
    }
}

void ExportRos::updateTopic()
{
    topic_ = readParameter<std::string>("topic");
    queue_ = readParameter<int>("queue");
    pub.reset();
}
