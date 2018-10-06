/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex
{
class Demux : public Node, public VariadicOutputs
{
public:
    Demux()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        selector_ = node_modifier.addInput<connection_types::GenericValueMessage<int>>("select");
        input_ = node_modifier.addInput<connection_types::AnyMessage>("message");

        VariadicOutputs::setupVariadic(node_modifier);
    }

    void setupParameters(Parameterizable& parameters)
    {
        VariadicOutputs::setupVariadicParameters(parameters);
    }

    void process()
    {
        int select = msg::getValue<int>(selector_);
        if (select < 0 || select >= static_cast<int>(variadic_outputs_.size()))
            throw std::runtime_error(std::string("demux index out of range: ") + std::to_string(select));

        TokenData::ConstPtr msg = msg::getMessage<TokenData>(input_);
        msg::publish(variadic_outputs_[select].get(), msg);
    }

    virtual csapex::Output* createVariadicOutput(csapex::TokenDataConstPtr type, const std::string& label) override
    {
        return VariadicOutputs::createVariadicOutput(makeEmpty<connection_types::AnyMessage>(), "message");
    }

private:
    Input* selector_;
    Input* input_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::Demux, csapex::Node)
