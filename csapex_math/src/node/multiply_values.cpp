
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class MultiplyValues : public Node, public csapex::VariadicInputs
{
public:
    MultiplyValues()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        setupVariadic(modifier);
        out_ = modifier.addOutput<AnyMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        setupVariadicParameters(params);
    }

    void process() override
    {
        std::size_t num_inputs = VariadicInputs::getVariadicInputCount();

        double res = 1.0;
        for (std::size_t i_inputs = 0; i_inputs < num_inputs; ++i_inputs) {
            InputPtr in = VariadicInputs::getVariadicInput(i_inputs);
            if (msg::isExactValue<int>(in.get())) {
                res *= msg::getValue<int>(in.get());

            } else if (msg::isValue<double>(in.get())) {
                if(msg::isExactMessage<double>(in.get())){
                    node_modifier_->setWarning("Multiplation for the inserted type is not implented. Value is casted to double!");
                } else{
                    node_modifier_->setNoError();
                }
                res *= msg::getValue<double>(in.get());
            }
        }

        InputPtr in = VariadicInputs::getVariadicInput(0);
        if (msg::isExactValue<int>(in.get())) {
            int out = (int)res;
            msg::publish(out_, out);
        } else if (msg::isValue<double>(in.get())) {
            msg::publish(out_, res);
        }
    }

private:
    Input* in_;
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::MultiplyValues, csapex::Node)
