
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class NegateNumber : public Node
{
public:
    NegateNumber()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addMultiInput<double, int>("a");
        out_ = modifier.addOutput<AnyMessage>("-a");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        if(msg::isMessage<connection_types::GenericValueMessage<double>>(in_)) {
            msg::publish(out_, msg::getValue<double>(in_) * -1);

        } else if(msg::isMessage<connection_types::GenericValueMessage<int>>(in_)) {
            msg::publish(out_, msg::getValue<int>(in_) * -1);

        } else {
            throw std::runtime_error("invalid input type");
        }

    }

private:
    Input* in_;
    Output* out_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::NegateNumber, csapex::Node)

