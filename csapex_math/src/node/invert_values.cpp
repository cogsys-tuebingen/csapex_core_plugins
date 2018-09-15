
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


class InvertValues : public Node
{
public:
    InvertValues()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<double>("a");
        out_ = modifier.addOutput<double>("1/a");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        double value = msg::getValue<double>(in_);

        double out = 1.0 / value;

        msg::publish(out_, out);
    }

private:
    Input* in_;
    Output* out_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::InvertValues, csapex::Node)

