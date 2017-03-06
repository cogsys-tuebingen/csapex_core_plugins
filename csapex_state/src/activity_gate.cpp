
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
namespace state
{

class ActivityGate : public Node, public VariadicIO
{
public:
    ActivityGate()
        : inverted_(false), active_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        VariadicIO::setupVariadic(modifier);

    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        VariadicIO::setupVariadicParameters(params);

        params.addParameter(param::ParameterFactory::declareBool("inverted",
                                                                 param::ParameterDescription("If inverted, messages are forwarded when the node is <b>not</b> active."),
                                                                 false),
                            inverted_);
    }

    void activation() override
    {
        active_ = true;
    }

    void deactivation() override
    {
        active_ = false;
    }

    void process() override
    {
        if(active_ ^ inverted_) {
            apex_assert(variadic_outputs_.size() == variadic_inputs_.size());

            for(std::size_t i = 0, n = variadic_inputs_.size(); i < n; ++i) {
                InputPtr in = variadic_inputs_.at(i);
                OutputPtr out = variadic_outputs_.at(i);

                msg::publish(out.get(), msg::getMessage(in.get()));
            }
        }
    }

    Input* createVariadicInput(TokenDataConstPtr type, const std::string& label, bool /*optional*/) override
    {
        VariadicOutputs::createVariadicOutput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label);
        return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
    }


    Output* createVariadicOutput(TokenDataConstPtr type, const std::string& label) override
    {
        VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
        return VariadicOutputs::createVariadicOutput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label);
    }

    void finishSetup() override
    {
        apex_assert(variadic_outputs_.size() == variadic_inputs_.size());
        if(getVariadicInputCount() == 0) {
            createVariadicInput(connection_types::makeEmpty<connection_types::AnyMessage>(), "", false);
        }
    }

private:
    bool inverted_;
    bool active_;
};

}
}

CSAPEX_REGISTER_CLASS(csapex::state::ActivityGate, csapex::Node)

