/// HEADER
#include "clock.h"

/// COMPONENT
#include <csapex_transform/time_stamp_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex_ros/ros_handler.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/assign/list_of.hpp>

CSAPEX_REGISTER_CLASS(csapex::Clock, csapex::Node)

using namespace csapex;

Clock::Clock()
{
    addTag(Tag::get("Time"));

    std::map<std::string, int> methods = boost::assign::map_list_of
            ("ros::Time::now()", (int) CURRENT)
            ("ros::Time(0)", (int) ZERO);
    addParameter(param::ParameterFactory::declareParameterSet("method", methods, (int) CURRENT));
}


void Clock::tick()
{
    ROSHandler::instance().waitForConnection();

    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    if(param<int>("method") == CURRENT) {
        time->value = ros::Time::now();
    } else {
        time->value = ros::Time(0);
    }
    output_->publish(time);
}

void Clock::process()
{

}

void Clock::setup()
{
    output_ = modifier_->addOutput<connection_types::TimeStampMessage>("Time");
}

