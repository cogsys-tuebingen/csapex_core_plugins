
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_transform/transform_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class ExtractFrames : public Node
{
public:
    ExtractFrames()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<TransformMessage>("Transform");
        out_frame_ = modifier.addOutput<std::string>("Frame");
        out_childframe_ = modifier.addOutput<std::string>("Child Frame");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        TransformMessage::ConstPtr trafo = msg::getMessage<TransformMessage>(in_);

        msg::publish(out_frame_, trafo->frame_id);
        msg::publish(out_childframe_, trafo->child_frame);
    }

private:
    Input* in_;
    Output* out_frame_;
    Output* out_childframe_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::ExtractFrames, csapex::Node)
