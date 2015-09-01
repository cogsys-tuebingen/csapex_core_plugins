/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_core_plugins/composite_message.h>

namespace csapex
{

class Composer : public Node
{
public:
    Composer()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        output_ = node_modifier.addOutput<connection_types::CompositeMessage>("Composed");
    }

    void setupParameters(Parameterizable& parameters)
    {
        addParameter(param::ParameterFactory::declareRange("inputs", 1, 10, 2, 1), [this](param::Parameter* p) {
            updateInputs();
        });
    }

    void process()
    {
        auto composite = std::make_shared<connection_types::CompositeMessage>();

        for(auto input : modifier_->getMessageInputs()) {
            ConnectionType::ConstPtr msg = msg::getMessage(input);
            if(std::dynamic_pointer_cast<connection_types::NoMessage const>(msg)) {
                return;
            }
            composite->value.push_back(msg);
        }

        msg::publish(output_, composite);
    }

    void updateInputs()
    {
        int input_count = readParameter<int>("inputs");

        std::vector<Input*> inputs = modifier_->getMessageInputs();
        int current_amount = inputs.size();

        if(current_amount > input_count) {
            for(int i = current_amount; i > input_count ; i--) {
                Input* in = inputs[i - 1];
                if(msg::isConnected(in)) {
//                    msg::disable(in);
                    break;
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
                modifier_->addOptionalInput<connection_types::AnyMessage>("Message");
            }
        }

    }

private:
    Output* output_;
};

}

CSAPEX_REGISTER_CLASS(csapex::Composer, csapex::Node)
