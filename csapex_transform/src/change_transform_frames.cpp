
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_transform/transform_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

class ChangeTransformFrames : public Node
{
public:
    ChangeTransformFrames()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<TransformMessage>("Transform");
        out_ = modifier.addOutput<TransformMessage>("Transform");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareText("frame_id", ""), frame_id_);
        params.addParameter(param::factory::declareText("child_frame_id", ""), child_frame_id_);
    }

    void process() override
    {
        TransformMessage::Ptr trafo = msg::getClonedMessage<TransformMessage>(in_);

        if(!frame_id_.empty()) {
            trafo->frame_id = frame_id_;
        }

        if(!child_frame_id_.empty()) {
            trafo->child_frame = child_frame_id_;
        }

        msg::publish(out_, trafo);
    }

private:
    Input* in_;
    Output* out_;

    std::string frame_id_;
    std::string child_frame_id_;
};

}

CSAPEX_REGISTER_CLASS(csapex::ChangeTransformFrames, csapex::Node)

