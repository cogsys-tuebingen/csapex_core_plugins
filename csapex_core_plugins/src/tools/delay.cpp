/// HEADER
#include "delay.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/output_progress_parameter.h>
#include <csapex/signal/event.h>
#include <csapex/msg/any_message.h>

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
    addParameter(csapex::param::ParameterFactory::declareRange<double>
                 ("delay",
                  csapex::param::ParameterDescription("Delay <b><span style='color: red'>in seconds</style></b> to wait after each message."),
                  0.0, 10.0, 1.0, 0.01));

    csapex::param::Parameter::Ptr p = csapex::param::ParameterFactory::declareOutputProgress("delay progress");
    progress_ = dynamic_cast<param::OutputProgressParameter*>(p.get());
    parameters.addParameter(p);
}

void Delay::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::AnyMessage>("Input");
    output_ = node_modifier.addOutput<connection_types::AnyMessage>("Delayed Input");

    delayed_forward_ = node_modifier.addEvent("delayed forwarded signal");
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

    apex_assert_hard(wait_time == readParameter<double>("delay") * 1000);
}

void Delay::process()
{
    TokenData::ConstPtr msg = msg::getMessage<TokenData>(input_);

    doSleep();

    msg::publish(output_, msg);
}

