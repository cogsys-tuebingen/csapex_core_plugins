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

CSAPEX_REGISTER_CLASS(csapex::Delay, csapex::Node)

using namespace csapex;

Delay::Delay()
    : input_(NULL), output_(NULL)
{
    addParameter(param::ParameterFactory::declareRange<double>
                 ("delay",
                  param::ParameterDescription("Delay <b><span style='color: red'>in seconds</style></b> to wait after each message."),
                  0.0, 10.0, 1.0, 0.1));
}

void Delay::setup()
{
    input_ = modifier_->addInput<connection_types::AnyMessage>("Input");
    output_ = modifier_->addOutput<connection_types::AnyMessage>("Delayed Input");
}

void Delay::process()
{
    ConnectionType::Ptr msg = input_->getMessage<ConnectionType>();

    long t = readParameter<double>("delay") * 1000;
    qt_helper::QSleepThread::msleep(t);

    output_->setType(input_->getType());
    output_->publish(msg);
}

