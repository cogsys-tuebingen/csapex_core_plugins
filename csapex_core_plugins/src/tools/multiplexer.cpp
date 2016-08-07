/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/no_message.h>
#include <csapex/model/token.h>
#include <csapex/serialization/yaml.h>

namespace csapex
{

class Multiplexer : public Node
{
public:
    Multiplexer()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        auto i = node_modifier.addDynamicInput<connection_types::AnyMessage>("series of messages");
        input = dynamic_cast<DynamicInput*> (i);
        output = node_modifier.addOutput<connection_types::GenericVectorMessage, TokenDataConstPtr>("multidimensional message");
    }

    void setupParameters(Parameterizable& parameters)
    {
    }

    void process()
    {
        std::vector<TokenPtr> msg = input->getMessageParts();
//        aerr << "size is " << msg.size() << std::endl;

        if(msg.empty()) {
            msg::publish(output, connection_types::makeEmpty<connection_types::NoMessage>());
        } else {
            std::shared_ptr<std::vector<TokenDataConstPtr>> composed(new std::vector<TokenDataConstPtr>());
//            composed->value = msg;
            for(auto& m : msg) {
                if(std::dynamic_pointer_cast<connection_types::MarkerMessage const>(m->getTokenData()) == nullptr) {
                    composed->push_back(m->getTokenData());
                }
            }

            msg::publish<connection_types::GenericVectorMessage, TokenDataConstPtr>(output, composed);
        }
    }

private:
    DynamicInput* input;
    Output* output;
};

}

CSAPEX_REGISTER_CLASS(csapex::Multiplexer, csapex::Node)
