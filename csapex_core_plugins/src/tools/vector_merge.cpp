/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_core_plugins/vector_message.h>

namespace csapex
{

class VectorMerge : public Node
{
public:
    VectorMerge()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        output = node_modifier.addOutput<connection_types::GenericVectorMessage>("merged vector");

        updateInputs();
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(csapex::param::ParameterFactory::declareRange("input count", 2, 10, 2, 1), std::bind(&VectorMerge::updateInputs, this));
    }

    void process()
    {
        // TODO: implement variadic io
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
                    msg= msg::getMessage<connection_types::GenericVectorMessage>(in);
                    for(std::size_t i = 0, total = msg->nestedValueCount(); i < total; ++i) {
                        result->addNestedValue(msg->nestedValue(i));
                    }
                }
            }
        }

        if(result) {
            msg::publish(output, result);
        }
    }


    void updateInputs()
    {
        int input_count = readParameter<int>("input count");

        std::vector<Input*> inputs = node_modifier_->getMessageInputs();
        int current_amount = inputs.size();

        if(current_amount > input_count) {
            for(int i = current_amount; i > input_count ; i--) {
                Input* in = inputs[i - 1];
                if(msg::isConnected(in)) {
                    msg::disable(in);
                } else {
                    node_modifier_->removeInput(msg::getUUID(in));
                }
            }
        } else {
            int to_add = input_count - current_amount;
            for(int i = 0 ; i < current_amount; i++) {
                msg::enable(inputs[i]);
            }
            for(int i = 0 ; i < to_add ; i++) {
                node_modifier_->addOptionalInput<connection_types::GenericVectorMessage>("Vector");
            }
        }

    }

private:
    Output* output;
};

}

CSAPEX_REGISTER_CLASS(csapex::VectorMerge, csapex::Node)
