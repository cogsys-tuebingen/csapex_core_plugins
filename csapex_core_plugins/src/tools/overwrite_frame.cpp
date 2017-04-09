
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class OverwriteFrame : public Node
{
public:
    OverwriteFrame()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");
        out_ = modifier.addOutput<AnyMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareText("frame_id", "frame_id"), frame_id_);
    }

    void process() override
    {
        TokenDataConstPtr message = msg::getMessage(in_);
        MessagePtr message_clone = message->cloneAs<Message>();
        if(message_clone) {
            message_clone->frame_id = frame_id_;
            msg::publish(out_, message_clone);
        }
    }

private:
    Input* in_;
    Output* out_;

    std::string frame_id_;
};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::OverwriteFrame, csapex::Node)

