/// HEADER
#include "delay.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/model/node_modifier.h>
#include <utils_param/output_progress_parameter.h>
#include <csapex/signal/trigger.h>

/// SYSTEM
#include <thread>

CSAPEX_REGISTER_CLASS(csapex::Delay, csapex::Node)

using namespace csapex;

Delay::Delay()
    : input_(nullptr), output_(nullptr)
{
}

void Delay::setupParameters(Parameterizable& parameters)
{
    addParameter(param::ParameterFactory::declareRange<double>
                 ("delay",
                  param::ParameterDescription("Delay <b><span style='color: red'>in seconds</style></b> to wait after each message."),
                  0.0, 10.0, 1.0, 0.1));

    param::Parameter::Ptr p = param::ParameterFactory::declareOutputProgress("delay progress");
    progress_ = dynamic_cast<param::OutputProgressParameter*>(p.get());
    parameters.addParameter(p);
}

void Delay::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::AnyMessage>("Input");
    output_ = node_modifier.addOutput<connection_types::AnyMessage>("Delayed Input");

    delayed_forward_ = node_modifier.addTrigger("delayed forwarded signal");
    delayed_slot_ = node_modifier.addSlot("delayed slot", [this]() {
        doSleep();
        delayed_forward_->trigger();
    });
}

void Delay::doSleep()
{
    long wait_time = readParameter<double>("delay") * 1000;
    long t = wait_time;

    while(t > 0) {
        progress_->setProgress(wait_time - t, wait_time);
        std::chrono::milliseconds dura(std::min(10l, t));
        std::this_thread::sleep_for(dura);
        t -= 10;
    }
    progress_->setProgress(wait_time, wait_time);
}

void Delay::process()
{
    ConnectionType::ConstPtr msg = msg::getMessage<ConnectionType>(input_);

    doSleep();

    msg::publish(output_, msg);
}

