
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/input.h>
#include <csapex/signal/event.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class MessageEvent : public Node
{
    enum class Type {
        END_OF_PROGRAM = 4,
        END_OF_SEQUENCE = 2,
        NO_MESSAGE = 1
    };

public:
    MessageEvent()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");

        event_ = modifier.addEvent("Signal");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        event_->triggerWith(in_->getToken());
    }

private:
    Input* in_;
    Event* event_;
};

} // csapex

CSAPEX_REGISTER_CLASS(csapex::MessageEvent, csapex::Node)

