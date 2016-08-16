/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex_core_plugins/duration_message.h>

using namespace csapex::connection_types;


namespace csapex
{

class CSAPEX_EXPORT_PLUGIN CalculateDuration : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_a_ = modifier.addInput<TimestampMessage>("a");
        in_b_ = modifier.addOptionalInput<TimestampMessage>("b");

        out_ = modifier.addOutput<DurationMessage>("duration(a-b)");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        TimestampMessage::ConstPtr msg_a(msg::getMessage<TimestampMessage>(in_a_));
        auto tp_a = msg_a->value;

        std::chrono::microseconds ms(0);

        if(msg::isConnected(in_b_)) {
            TimestampMessage::ConstPtr msg_b(msg::getMessage<TimestampMessage>(in_b_));
            ms = (tp_a - msg_b->value);

        } else {
            connection_types::TimestampMessage::Tp now;
            auto hires = std::chrono::high_resolution_clock::now();
            now = std::chrono::time_point_cast<std::chrono::microseconds>(hires);
            ms = now - tp_a;
//            ms = tp_a.time_since_epoch();
//            ms = now.time_since_epoch();
            ainfo << ms.count() << std::endl;
        }

        DurationMessage::Ptr res = std::make_shared<DurationMessage>(ms);
        ainfo << res->value.count() << std::endl;

        msg::publish(out_, res);
    }

private:
    Input* in_a_;
    Input* in_b_;
    Output* out_;
};


}

CSAPEX_REGISTER_CLASS(csapex::CalculateDuration, csapex::Node)


