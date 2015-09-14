/// HEADER
#include "time_offset.h"

/// COMPONENT
#include <csapex_ros/time_stamp_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::TimeOffset, csapex::Node)

using namespace csapex;

TimeOffset::TimeOffset()
{
}

void TimeOffset::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange<double>("offset", -5000.0, 5000.0, 0.0, 0.5));
}


void TimeOffset::process()
{
    connection_types::TimeStampMessage::ConstPtr in = msg::getMessage<connection_types::TimeStampMessage>(input_);
    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);

    double offset = readParameter<double>("offset");

    if(in->value.toNSec() != 0) {
        aerr << in->value.toNSec() << " + " << offset << " * " << 1e6 << " = " << (in->value.toNSec() + offset * 1e6) << std::endl,
                time->value = time->value.fromNSec((in->value.toNSec() + offset * 1000000));
        modifier_->setNoError();
    } else {
        modifier_->setWarning("Time is 0, using current time as base");
        ros::Time now = ros::Time::now();
        time->value = now - ros::Duration(0, offset * 1000000);
    }
    msg::publish(output_, time);
}

void TimeOffset::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::TimeStampMessage>("Time");
    output_ = node_modifier.addOutput<connection_types::TimeStampMessage>("Time");
}
