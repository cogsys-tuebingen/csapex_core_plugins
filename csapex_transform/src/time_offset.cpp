/// HEADER
#include "time_offset.h"

/// COMPONENT
#include <csapex_transform/time_stamp_message.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::TimeOffset, csapex::Node)

using namespace csapex;

TimeOffset::TimeOffset()
{
    addParameter(param::ParameterFactory::declareRange<double>("offset", -5000.0, 5000.0, 0.0, 0.5));
}


void TimeOffset::process()
{
    connection_types::TimeStampMessage::Ptr in = input_->getMessage<connection_types::TimeStampMessage>();
    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);

    double offset = readParameter<double>("offset");

    if(in->value.toNSec() != 0) {
        aerr << in->value.toNSec() << " + " << offset << " * " << 1e6 << " = " << (in->value.toNSec() + offset * 1e6) << std::endl,
                time->value = time->value.fromNSec((in->value.toNSec() + offset * 1000000));
        setError(false);
    } else {
        setError(true, "Time is 0, using current time as base", EL_WARNING);
        ros::Time now = ros::Time::now();
        time->value = now - ros::Duration(0, offset * 1000000);
    }
    output_->publish(time);
}

void TimeOffset::setup()
{
    input_ = modifier_->addInput<connection_types::TimeStampMessage>("Time");
    output_ = modifier_->addOutput<connection_types::TimeStampMessage>("Time");
}
