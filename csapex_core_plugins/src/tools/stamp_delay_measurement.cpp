/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex::connection_types;

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN StampDelayMeasurement : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Message");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        Message::ConstPtr msg(msg::getMessage<Message>(in_));

        auto stamp = std::chrono::microseconds(long(msg->stamp_micro_seconds));
        auto now = std::chrono::high_resolution_clock::now().time_since_epoch();

        auto delta = now - stamp;

        ainfo << "Current time: " << now.count() << "\tStamp: " << stamp.count() << "\tDelay: " << delta.count() * 1e-3 << " milliseconds" << std::endl;
    }

private:
    Input* in_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::StampDelayMeasurement, csapex::Node)
