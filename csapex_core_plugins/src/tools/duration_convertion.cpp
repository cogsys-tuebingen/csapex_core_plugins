/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/duration_message.h>

using namespace csapex::connection_types;

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN DurationConvertion : public Node
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

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::DurationConvertion, csapex::Node)
