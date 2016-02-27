/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_core_plugins/duration_message.h>

/// SYSTEM
#include <ros/time.h>

using namespace csapex::connection_types;


namespace csapex
{

class DurationConvertion : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<DurationMessage>("duration");
        out_ = modifier.addOutput<double>("double");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        DurationMessage::ConstPtr dur = msg::getMessage<DurationMessage>(in_);
        std::chrono::microseconds s = dur->value;
        msg::publish(out_, s.count() * 1e-6);
    }

private:
    Input* in_;
    Output* out_;
};


}

CSAPEX_REGISTER_CLASS(csapex::DurationConvertion, csapex::Node)


