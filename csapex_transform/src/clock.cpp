/// HEADER
#include "clock.h"

/// COMPONENT
#include <csapex_core_plugins/timestamp_message.h>

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
        {"ros::Time(0)", (int) ZERO},
        {"std::chrono::system_clock::now();", (int) CHRONO}
    };
    parameters.addParameter(csapex::param::factory::declareParameterSet("method", methods, (int) CURRENT));
}


void Clock::process()
{
    connection_types::TimestampMessage::Tp tp;

    switch(readParameter<int>("method")) {
    case CURRENT:
    {
        ROSHandler& ros_handler = ROSHandler::instance();

        if(!ros_handler.isConnected()) {
            node_modifier_->setWarning("Waiting for ROS connection");
            ros_handler.waitForConnection();
        }

        apex_assert(ros_handler.isConnected());

        ros_handler.nh();

        std::chrono::nanoseconds ns(ros::Time::now().toNSec());
        auto ms = std::chrono::duration_cast<std::chrono::microseconds>(ns);
        tp = connection_types::TimestampMessage::Tp(ms);
    }
    break;
    case ZERO:
    {
        tp = connection_types::TimestampMessage::Tp(std::chrono::microseconds(0));
    }
        break;
    default:
    case CHRONO:
    {
        auto hires = std::chrono::high_resolution_clock::now();
        tp = std::chrono::time_point_cast<std::chrono::microseconds>(hires);
    }
        break;
    }

    connection_types::TimestampMessage::Ptr time(new connection_types::TimestampMessage(tp));
    msg::publish(output_, time);
}

void Clock::setup(NodeModifier& node_modifier)
{
    output_ = node_modifier.addOutput<connection_types::TimestampMessage>("Time");
}

