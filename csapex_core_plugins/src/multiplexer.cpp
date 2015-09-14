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

class Multiplexer : public Node
{
public:
    Multiplexer()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        auto i = node_modifier.addDynamicInput<connection_types::AnyMessage>("series of messages");
        input = dynamic_cast<DynamicInput*> (i);
        output = node_modifier.addOutput<connection_types::AnyMessage>("multidimensional message");
    }

    void setupParameters(Parameterizable& parameters)
    {
    }

    void process()
    {
        std::vector<ConnectionTypeConstPtr> msg = input->getMessageParts();
//        aerr << "size is " << msg.size() << std::endl;

        if(msg.empty()) {
            msg::publish(output, connection_types::makeEmpty<connection_types::NoMessage>());
        } else {
            connection_types::VectorMessage::Ptr composed(new connection_types::VectorMessage);
//            composed->value = msg;
            for(auto& m : msg) {
                if(std::dynamic_pointer_cast<connection_types::NoMessage const>(m) == nullptr) {
                    composed->value.push_back(m);
                }
            }

            msg::publish(output, composed);
        }
    }

private:
    DynamicInput* input;
    Output* output;
};

}

CSAPEX_REGISTER_CLASS(csapex::Multiplexer, csapex::Node)
