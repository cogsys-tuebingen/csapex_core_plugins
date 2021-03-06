/// HEADER
#include "holdable_buffer.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::HoldableBuffer, csapex::Node)

using namespace csapex;

HoldableBuffer::HoldableBuffer() : in_(nullptr), out_(nullptr), buffer_(1)
{
}

void HoldableBuffer::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
    out_ = node_modifier.addOutput<connection_types::AnyMessage>("Same as input");
}

void HoldableBuffer::process()
{
    TokenData::ConstPtr msg = msg::getMessage<TokenData>(in_);

    unsigned int size = readParameter<int>("buffer size");
    bool hold = readParameter<bool>("hold");
    param::RangeParameter::Ptr range = getParameter<param::RangeParameter>("out idx");
    range->setMax<int>(buffer_.size() - 1);
    setParameterEnabled("out idx", hold);

    TokenData::ConstPtr out;

    if (hold && buffer_.size() > 0) {
        int idx = readParameter<int>("out idx");
        out = buffer_.at(idx);
    } else {
        if (buffer_.size() < size) {
            buffer_.push_back(msg);
            out = buffer_.front();
        } else if (buffer_.size() > size) {
            out = buffer_.front();
            buffer_.pop_front();
        } else {
            out = buffer_.front();
            buffer_.pop_front();
            buffer_.push_back(msg);
        }
    }

    if (out) {
        msg::publish(out_, out);
    }
}

void HoldableBuffer::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("buffer size", 1, 50, 1, 1));
    parameters.addParameter(csapex::param::factory::declareBool("hold", false));
    parameters.addParameter(csapex::param::factory::declareRange("out idx", 0, (int)buffer_.size() - 1, 0, 1));
    setParameterEnabled("out idx", false);
}
