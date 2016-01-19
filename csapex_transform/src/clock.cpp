/// HEADER
#include "clock.h"

/// COMPONENT
#include <csapex_ros/time_stamp_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_ros/ros_handler.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Clock, csapex::Node)

using namespace csapex;

Clock::Clock()
{
}

void Clock::setupParameters(Parameterizable &parameters)
{
    std::map<std::string, int> methods = {
        {"ros::Time::now()", (int) CURRENT},
        {"ros::Time(0)", (int) ZERO}
    };
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet("method", methods, (int) CURRENT));
}


void Clock::tick()
{
    getRosHandler().waitForConnection();

    if(!getRosHandler().isConnected()) {
        node_modifier_->setWarning("No ROS connection");
        return;
    }

    getRosHandler().nh();

    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    if(readParameter<int>("method") == CURRENT) {
        time->value = ros::Time::now();
    } else {
        time->value = ros::Time(0);
    }
    msg::publish(output_, time);
}

void Clock::setupROS()
{

}
void Clock::processROS()
{

}

void Clock::setup(NodeModifier& node_modifier)
{
    output_ = node_modifier.addOutput<connection_types::TimeStampMessage>("Time");
}

