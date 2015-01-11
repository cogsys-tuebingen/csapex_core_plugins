/// HEADER
#include "delay.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/model/node_modifier.h>
#include <utils_param/output_progress_parameter.h>

CSAPEX_REGISTER_CLASS(csapex::Delay, csapex::Node)

using namespace csapex;

Delay::Delay()
    : input_(NULL), output_(NULL)
{
}

void Delay::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange<double>
                 ("delay",
                  param::ParameterDescription("Delay <b><span style='color: red'>in seconds</style></b> to wait after each message."),
                  0.0, 10.0, 1.0, 0.1));

    param::Parameter::Ptr p = param::ParameterFactory::declareOutputProgress("delay progress");
    progress_ = dynamic_cast<param::OutputProgressParameter*>(p.get());
    addParameter(p);
}

void Delay::setup()
{
    input_ = modifier_->addInput<connection_types::AnyMessage>("Input");
    output_ = modifier_->addOutput<connection_types::AnyMessage>("Delayed Input");
}

void Delay::process()
{
    ConnectionType::ConstPtr msg = input_->getMessage<ConnectionType>();

    long wait_time = readParameter<double>("delay") * 1000;
    long t = wait_time;

    while(t > 0) {
        progress_->setProgress(wait_time - t, wait_time);
        qt_helper::QSleepThread::msleep(std::min(10l, t));
        t -= 10;
    }
    progress_->setProgress(wait_time, wait_time);

    output_->setType(input_->getType());
    output_->cloneAndPublish(msg);
}

