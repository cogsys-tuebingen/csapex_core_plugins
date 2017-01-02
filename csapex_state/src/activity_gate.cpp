
/// PROJECT
#include <csapex/model/node.h>
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

class ActivityGate : public Node
{
public:
    ActivityGate()
        : inverted_(false), active_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");
        out_ = modifier.addOutput<AnyMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
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
            auto message = msg::getMessage(in_);
            apex_assert(message);
            msg::publish(out_, message);
        }
    }

private:
    Input* in_;
    Output* out_;

    bool inverted_;
    bool active_;
};

}
}

CSAPEX_REGISTER_CLASS(csapex::state::ActivityGate, csapex::Node)

