/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;

namespace csapex
{
template <typename T>
class NumberInput : public Node
{
public:
    NumberInput()
    {
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        out_ = node_modifier.addOutput<T>(type2name(typeid(T)));
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(csapex::param::factory::declareValue<T>("value", (T)0.0));
    }

    void process() override
    {
        T val = readParameter<T>("value");
        msg::publish(out_, val);
    }

private:
    Output* out_;
};

}  // namespace csapex

namespace csapex
{
typedef NumberInput<int> IntInput;
typedef NumberInput<double> DoubleInput;
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::IntInput, csapex::Node)
CSAPEX_REGISTER_CLASS(csapex::DoubleInput, csapex::Node)
