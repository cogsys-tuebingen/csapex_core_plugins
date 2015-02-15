/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

namespace csapex
{

class Demultiplexer : public Node
{
public:
    void setup()
    {
        input = modifier_->addInput<connection_types::AnyMessage>("multidimensional message");
        output = modifier_->addOutput<connection_types::AnyMessage>("demultiplexed message");
    }

    void process()
    {

    }

private:
    Input* input;
    Output* output;
};

}

CSAPEX_REGISTER_CLASS(csapex::Demultiplexer, csapex::Node)
