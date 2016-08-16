/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN VectorMerge : public Node, public VariadicInputs
{
public:
    VectorMerge()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        setupVariadic(node_modifier);

        output = node_modifier.addOutput<connection_types::GenericVectorMessage>("merged vector");
    }

    void setupParameters(Parameterizable& parameters)
    {
        setupVariadicParameters(parameters);
    }

    void process()
    {
        connection_types::GenericVectorMessage::Ptr result;

        bool first = true;
        std::vector<Input*> inputs = node_modifier_->getMessageInputs();
        for(std::size_t i = 0 ; i < inputs.size() ; i++) {
            Input *in = inputs[i];
            if(msg::hasMessage(in)) {
                connection_types::GenericVectorMessage::ConstPtr msg;
                if(first) {
                    result =  msg::getClonedMessage<connection_types::GenericVectorMessage>(in);
                    first = false;
                } else {
                    msg = msg::getMessage<connection_types::GenericVectorMessage>(in);
                    for(std::size_t j = 0, total = msg->nestedValueCount(); j < total; ++j) {
                        result->addNestedValue(msg->nestedValue(j));
                    }
                }
            }
        }

        if(result) {
            msg::publish(output, result);
        }
    }

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override
    {
        return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::GenericVectorMessage>(), label.empty() ? "Vector" : label, getVariadicInputCount() == 0 ? false : true);
    }

private:
    Output* output;
};

}

CSAPEX_REGISTER_CLASS(csapex::VectorMerge, csapex::Node)
