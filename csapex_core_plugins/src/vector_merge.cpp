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
        output = node_modifier.addOutput<connection_types::VectorMessage>("merged vector");

        updateInputs();
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(csapex::param::ParameterFactory::declareRange("input count", 2, 10, 2, 1), std::bind(&VectorMerge::updateInputs, this));
    }

    void process()
    {
        connection_types::VectorMessage::Ptr result(new connection_types::VectorMessage);

        bool first = true;
        std::vector<Input*> inputs = modifier_->getMessageInputs();
        for(std::size_t i = 0 ; i < inputs.size() ; i++) {
            Input *in = inputs[i];
            if(msg::hasMessage(in)) {
                connection_types::VectorMessage::ConstPtr msg = msg::getMessage<connection_types::VectorMessage>(in);
                if(first) {
                    result->stamp_micro_seconds = msg->stamp_micro_seconds;
                    first = false;
                }
                auto v = msg->value;
                result->value.insert(result->value.end(), v.begin(), v.end());
            }
        }

        msg::publish(output, result);
    }


    void updateInputs()
    {
        int input_count = readParameter<int>("input count");

        std::vector<Input*> inputs = modifier_->getMessageInputs();
        int current_amount = inputs.size();

        if(current_amount > input_count) {
            for(int i = current_amount; i > input_count ; i--) {
                Input* in = inputs[i - 1];
                if(msg::isConnected(in)) {
                    msg::disable(in);
                } else {
                    modifier_->removeInput(msg::getUUID(in));
                }
            }
        } else {
            int to_add = input_count - current_amount;
            for(int i = 0 ; i < current_amount; i++) {
                msg::enable(inputs[i]);
            }
            for(int i = 0 ; i < to_add ; i++) {
                modifier_->addOptionalInput<connection_types::VectorMessage>("Vector");
            }
        }

    }

private:
    DynamicInput* input;
    Output* output;
};

}

CSAPEX_REGISTER_CLASS(csapex::VectorMerge, csapex::Node)
