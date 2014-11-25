/// HEADER
#include "holdable_buffer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <utils_param/range_parameter.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::HoldableBuffer, csapex::Node)

using namespace csapex;

HoldableBuffer::HoldableBuffer() :
    in_(NULL),
    out_(NULL),
    buffer_(1)
{
}

void HoldableBuffer::setup()
{
    in_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
    out_ = modifier_->addOutput<connection_types::AnyMessage>("Same as input");
}

void HoldableBuffer::process()
{
    ConnectionType::Ptr msg = in_->getMessage<ConnectionType>();

    unsigned int size = readParameter<int>("buffer size");
    bool hold = readParameter<bool>("hold");
    param::RangeParameter::Ptr range = getParameter<param::RangeParameter>("out idx");
    range->setMax<int>(buffer_.size() - 1);
    setParameterEnabled("out idx", hold);

    ConnectionType::Ptr out;

    if(hold && buffer_.size() > 0) {
        int idx = readParameter<int>("out idx");
        out = buffer_.at(idx);
    } else {
        if(buffer_.size() < size) {
            buffer_.push_back(msg);
            out = buffer_.front();
        } else if(buffer_.size() > size) {
            out = buffer_.front();
            buffer_.pop_front();
        } else {
            out = buffer_.front();
            buffer_.pop_front();
            buffer_.push_back(msg);
        }
    }

    if(out_->getType()->rawName() != msg->rawName()) {
        out_->setType(msg->toType());
    }

    if(out) {
        out_->publish(out);
    }

}

void HoldableBuffer::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("buffer size",1, 50, 1, 1));
    addParameter(param::ParameterFactory::declareBool("hold", false));
    addParameter(param::ParameterFactory::declareRange("out idx", 0, (int) buffer_.size() - 1, 0 , 1));
    setParameterEnabled("out idx", false);
}
