/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/io.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_value_message.hpp>

namespace csapex
{

class Mux : public Node, public VariadicInputs
{
public:
    Mux()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        selector_ = node_modifier.addOptionalInput<connection_types::GenericValueMessage<int>>("select");
        VariadicInputs::setupVariadic(node_modifier);

        output_ = node_modifier.addOutput<connection_types::AnyMessage>("message");
    }

    void setupParameters(Parameterizable& parameters)
    {
        VariadicInputs::setupVariadicParameters(parameters);
    }

    void process()
    {
        TokenData::ConstPtr msg;

        if (msg::hasMessage(selector_)) {
            int select = msg::getValue<int>(selector_);
            if (select < 0 || select >= static_cast<int>(variadic_inputs_.size()))
                throw std::runtime_error(std::string("mux index out of range: ") + std::to_string(select));

            msg = msg::getMessage<TokenData>(variadic_inputs_[select].get());
        } else {
            for (auto input : variadic_inputs_) {
                if (msg::hasMessage(input.get())) {
                    if (!msg)
                        msg = msg::getMessage<TokenData>(input.get());
                    else
                        throw std::runtime_error("more than one message available without explicit selection");
                }
            }
        }

        if (msg)
            msg::publish(output_, msg);
    }

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override
    {
        return VariadicInputs::createVariadicInput(makeEmpty<connection_types::AnyMessage>(), "message", true);
    }

private:
    Input* selector_;
    Output* output_;
};

}

CSAPEX_REGISTER_CLASS(csapex::Mux, csapex::Node)
