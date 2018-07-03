
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>
#include <csapex/model/variadic_io.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class Join : public Node, public VariadicIO
{
public:
    Join()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        VariadicIO::setupVariadic(modifier);
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        VariadicIO::setupVariadicParameters(params);
    }

    void process() override
    {
        apex_assert(variadic_outputs_.size() == variadic_inputs_.size());

        for(std::size_t i = 0, n = variadic_inputs_.size(); i < n; ++i) {
            InputPtr in = variadic_inputs_.at(i);
            OutputPtr out = variadic_outputs_.at(i);

            msg::publish(out.get(), msg::getMessage(in.get()));
        }
    }

    Input* createVariadicInput(TokenDataConstPtr type, const std::string& label, bool /*optional*/) override
    {
        VariadicOutputs::createVariadicOutput(makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label);
        return VariadicInputs::createVariadicInput(makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
    }


    Output* createVariadicOutput(TokenDataConstPtr type, const std::string& label) override
    {
        VariadicInputs::createVariadicInput(makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
        return VariadicOutputs::createVariadicOutput(makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label);
    }

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::Join, csapex::Node)

