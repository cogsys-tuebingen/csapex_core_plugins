/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN VectorGenerate : public Node, public VariadicInputs
{
public:
    VectorGenerate()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        setupVariadic(node_modifier);

        output = node_modifier.addOutput<connection_types::GenericVectorMessage>("vector");
    }

    void setupParameters(Parameterizable& parameters)
    {
        setupVariadicParameters(parameters);
    }

    void process()
    {
        connection_types::GenericVectorMessage::Ptr result;

        std::vector<Input*> inputs = node_modifier_->getMessageInputs();
        for(std::size_t i = 0 ; i < inputs.size() ; i++) {
            Input *in = inputs[i];
            if(msg::hasMessage(in)) {
                TokenData::ConstPtr m =  msg::getMessage(in);

                if(!result) {
                    if(std::dynamic_pointer_cast<connection_types::GenericValueMessage<double> const>(m)) {
                        result = connection_types::GenericVectorMessage::make<double>();
                    } else if(std::dynamic_pointer_cast<connection_types::GenericValueMessage<int> const>(m)) {
                        result = connection_types::GenericVectorMessage::make<int>();
                    } else if(std::dynamic_pointer_cast<connection_types::GenericValueMessage<std::string> const>(m)) {
                        result = connection_types::GenericVectorMessage::make<std::string>();
                    } else {
                        result = connection_types::GenericVectorMessage::make(m);
                    }
                }

                if(result) {
                    result->addNestedValue(m);
                }
            }
        }

        if(result) {
            msg::publish(output, result);
        }
    }

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override
    {
        return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
    }

private:
    Output* output;
};

}

CSAPEX_REGISTER_CLASS(csapex::VectorGenerate, csapex::Node)
