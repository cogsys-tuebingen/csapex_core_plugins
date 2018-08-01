/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_core_plugins/composite_message.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/marker_message.h>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN Synchronizer : public Node
{
public:
    Synchronizer()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
    }

    void setupParameters(Parameterizable& parameters)
    {
        addParameter(csapex::param::factory::declareRange("inputs", 1, 10, 2, 1), [this](csapex::param::Parameter* p) {
            updateInputs();
        });
    }

    void process()
    {
    }

    void updateInputs()
    {
        int input_count = readParameter<int>("inputs");

        std::vector<InputPtr> inputs = node_modifier_->getMessageInputs();
        int current_amount = inputs.size();

        if(current_amount > input_count) {
            for(int i = current_amount; i > input_count ; i--) {
                Input* in = inputs[i - 1].get();
                if(msg::isConnected(in)) {
                    break;
                } else {
                    node_modifier_->removeInput(msg::getUUID(in));
                }
            }
        } else {
            int to_add = input_count - current_amount;
            for(int i = 0 ; i < current_amount; i++) {
                msg::enable(inputs[i].get());
            }
            for(int i = 0 ; i < to_add ; i++) {
                node_modifier_->addOptionalInput<connection_types::AnyMessage>("Message");
            }
        }

    }
};

}

CSAPEX_REGISTER_CLASS(csapex::Synchronizer, csapex::Node)
