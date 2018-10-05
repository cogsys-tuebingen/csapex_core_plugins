
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
class GenerateRelativeTransform : public Node
{
public:
    GenerateRelativeTransform() : init_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<TransformMessage>("Transform (absolute)");
        out_ = modifier.addOutput<TransformMessage>("Transform (relative)");

        modifier.addSlot("reset", [this]() { init_ = false; });
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        TransformMessage::Ptr trafo = msg::getClonedMessage<TransformMessage>(in_);
        const tf::Transform& odom_T_base_link = trafo->value;

        if (!init_) {
            init_T_odom_ = odom_T_base_link.inverse();
            init_ = true;
        }

        trafo->value = init_T_odom_ * odom_T_base_link;
        msg::publish(out_, trafo);
    }

private:
    Input* in_;
    Output* out_;

    tf::Transform init_T_odom_;

    bool init_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::GenerateRelativeTransform, csapex::Node)
