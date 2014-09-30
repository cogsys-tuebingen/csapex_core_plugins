/// HEADER
#include "double_buffer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::DoubleBuffer, csapex::Node)

using namespace csapex;

DoubleBuffer::DoubleBuffer()
    : input_(NULL), output_(NULL), dirty_(false)
{
    addParameter(param::ParameterFactory::declareBool("synchronized", true));
}

void DoubleBuffer::setup()
{
    input_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
    output_ = modifier_->addOutput<connection_types::AnyMessage>("Same as input");
}

void DoubleBuffer::process()
{
    ConnectionType::Ptr msg = input_->getMessage<ConnectionType>();

    buffer_back_ = msg->clone();

    swapBuffers();

    if(output_->getType()->name() != msg->name()) {
        output_->setType(msg->toType());
    }

    dirty_ = true;
}

void DoubleBuffer::swapBuffers()
{
    QMutexLocker lock(&mutex_);

    buffer_front_ = buffer_back_;
    buffer_back_.reset();
}

void DoubleBuffer::tick()
{
    if(!dirty_ && readParameter<bool>("synchronized")) {
        return;
    }

    if(!buffer_front_) {
        return;
    }

    ConnectionType::Ptr msg;
    {
        QMutexLocker lock(&mutex_);
        msg = buffer_front_;
    }

    output_->publish(msg);
    dirty_ = false;
}
