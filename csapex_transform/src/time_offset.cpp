/// HEADER
#include "time_offset.h"

/// COMPONENT
#include <csapex_core_plugins/timestamp_message.h>

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
    parameters.addParameter(csapex::param::factory::declareRange<double>("offset", -5.0, 5.0, 0.0, 0.001));
}


void TimeOffset::process()
{
    connection_types::TimestampMessage::ConstPtr in = msg::getMessage<connection_types::TimestampMessage>(input_);
    connection_types::TimestampMessage::Ptr time(new connection_types::TimestampMessage);

    double offset = readParameter<double>("offset");

    auto t = in->value;

    if(t.time_since_epoch().count() == 0) {
        node_modifier_->setWarning("Time is 0, using current time as base");
        auto hires = std::chrono::high_resolution_clock::now();
        t = std::chrono::time_point_cast<std::chrono::microseconds>(hires);
    } else {
        node_modifier_->setNoError();
    }
    time->value = connection_types::TimestampMessage::Tp(t + std::chrono::microseconds(long(offset * 1e6)));
    msg::publish(output_, time);
}

void TimeOffset::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::TimestampMessage>("Time");
    output_ = node_modifier.addOutput<connection_types::TimestampMessage>("Time");
}
