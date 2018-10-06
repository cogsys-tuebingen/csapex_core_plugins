
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
class Switch : public Node
{
public:
    Switch()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");
        out_true_ = modifier.addOutput<AnyMessage>("True");
        out_false_ = modifier.addOutput<AnyMessage>("False");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareBool("predicate",
                                                        param::ParameterDescription("hint: make the parameter <b>connectable</b> "
                                                                                    "for dynamic switching."),
                                                        true),
                            predicate_);
    }

    void process() override
    {
        auto message = msg::getMessage(in_);
        apex_assert(message);

        if (predicate_) {
            msg::publish(out_true_, message);

        } else {
            msg::publish(out_false_, message);
        }
    }

private:
    Input* in_;
    Output* out_true_;
    Output* out_false_;

    bool predicate_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::Switch, csapex::Node)
