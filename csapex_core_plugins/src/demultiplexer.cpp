/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_value_message.hpp>

namespace csapex
{

class Demultiplexer : public Node
{
public:
    void setup(csapex::NodeModifier& node_modifier)
    {
        input = node_modifier.addInput<connection_types::AnyMessage>("multidimensional message");
        output = node_modifier.addDynamicOutput<connection_types::AnyMessage>("demultiplexed message");
    }

    void process()
    {
        ConnectionTypeConstPtr msg = msg::getMessage(input);
        if(msg->isContainer()) {
            std::size_t size = msg->nestedValueCount();

            for(std::size_t i = 0; i < size; ++i) {
                ConnectionTypeConstPtr nested = msg->nestedValue(i);
                msg::publish(output, nested);
            }

        } else {
            msg::publish(output, msg);
        }
    }

private:
    Input* input;
    Output* output;
};

}

CSAPEX_REGISTER_CLASS(csapex::Demultiplexer, csapex::Node)
