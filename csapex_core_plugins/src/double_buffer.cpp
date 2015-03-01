/// HEADER
#include "double_buffer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::DoubleBuffer, csapex::Node)

using namespace csapex;

DoubleBuffer::DoubleBuffer()
    : input_(nullptr), output_(nullptr), dirty_(false)
{
}

void DoubleBuffer::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareBool("synchronized", true));
}

void DoubleBuffer::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
    output_ = node_modifier.addOutput<connection_types::AnyMessage>("Same as input");
}

void DoubleBuffer::process()
{
    ConnectionType::ConstPtr msg = msg::getMessage<ConnectionType>(input_);

    buffer_back_ = msg->clone();

    swapBuffers();

    dirty_ = true;
}

void DoubleBuffer::swapBuffers()
{
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
    msg = buffer_front_;

    msg::publish(output_, msg);
    dirty_ = false;
}
