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

#if WIN32
#undef min
#undef max
#endif


Delay::Delay()
    : input_(nullptr), output_(nullptr)
{
}

Delay::~Delay()
{
    if(future.valid()) {
        future.wait();
    }
}

void Delay::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<double>
                            ("delay",
                             csapex::param::ParameterDescription("Delay <b><span style='color: red'>in seconds</style></b> to wait after each message."),
                             0.0, 10.0, 1.0, 0.01));

    parameters.addParameter(csapex::param::ParameterFactory::declareBool
                            ("blocking",
                             csapex::param::ParameterDescription("If true, the node sleeps for the given time, not giving up resources to other nodes in "
                                                                 "the same thread group. Otherwise it uses signals to yield the CPU."),
                             false),
                            blocking_);

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
        if(future.valid()) {
            future.wait();
        }

        if(blocking_) {
            delayEvent();

        } else {
            if(delayed_forward_) {
                future = std::async(std::launch::async, [this]{
                    delayEvent();
                });
            }
        }

    });
}

void Delay::tearDown()
{
    delayed_forward_ = nullptr;
    output_ = nullptr;
}

bool Delay::isAsynchronous() const
{
    return true;
}

void Delay::doSleep()
{
    long wait_time = readParameter<double>("delay") * 1000;
    long t = wait_time;
    long step = 100l;

    while(t > 0) {
        progress_->setProgress(wait_time - t, wait_time);
        std::chrono::milliseconds dura(std::min(step, t));
        std::this_thread::sleep_for(dura);
        t -= step;
    }
    progress_->setProgress(wait_time, wait_time);
}

void Delay::process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters,
                    Continuation continuation)
{
    if(future.valid()) {
        future.wait();
    }

    if(blocking_) {
        delayInput(continuation);

    } else {
        future = std::async(std::launch::async, [this, continuation]{
            delayInput(continuation);
        });
    }
}

void Delay::delayInput(Continuation continuation)
{
    TokenData::ConstPtr msg = msg::getMessage<TokenData>(input_);
    doSleep();
    if(output_) {
        msg::publish(output_, msg);
        continuation([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
    }
}

void Delay::delayEvent()
{
    doSleep();
    if(delayed_forward_) {
        delayed_forward_->trigger();
    }
}
