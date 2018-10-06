
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
namespace state
{
class ActivitySwitch : public Node
{
public:
    ActivitySwitch()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");
        out_active_ = modifier.addOutput<AnyMessage>("Active");
        out_inactive_ = modifier.addOutput<AnyMessage>("Inactive");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
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
        auto message = msg::getMessage(in_);
        apex_assert(message);

        if (active_) {
            msg::publish(out_active_, message);

        } else {
            msg::publish(out_inactive_, message);
        }
    }

private:
    Input* in_;
    Output* out_active_;
    Output* out_inactive_;

    bool active_;
};

}  // namespace state
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::state::ActivitySwitch, csapex::Node)
