
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/timestamp_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class OverwriteTimestamp : public Node
{
public:
    OverwriteTimestamp()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");
        in_time_ = modifier.addInput<TimestampMessage>("Time");
        out_ = modifier.addOutput<AnyMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        TokenDataConstPtr message = msg::getMessage(in_);
        MessagePtr message_clone = message->cloneAs<Message>();
        if (message_clone) {
            TimestampMessage::ConstPtr time = msg::getMessage<TimestampMessage>(in_time_);

            message_clone->stamp_micro_seconds = time->stamp_micro_seconds;
            msg::publish(out_, message_clone);
        }
    }

private:
    Input* in_;
    Input* in_time_;
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::OverwriteTimestamp, csapex::Node)
