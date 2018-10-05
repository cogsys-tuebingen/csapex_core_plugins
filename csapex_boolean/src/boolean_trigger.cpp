
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class BooleanTrigger : public Node
{
public:
    BooleanTrigger()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<bool>("Input");

        true_ = modifier.addEvent("True");
        false_ = modifier.addEvent("False");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        bool value = msg::getValue<bool>(in_);

        if (value) {
            true_->trigger();
        } else {
            false_->trigger();
        }
    }

private:
    Input* in_;
    Event* true_;
    Event* false_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::BooleanTrigger, csapex::Node)
