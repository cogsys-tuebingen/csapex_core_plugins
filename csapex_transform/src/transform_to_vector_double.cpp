
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_transform/transform_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class TransformToVectorDouble : public Node
{
public:
    TransformToVectorDouble()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<TransformMessage>("Input");
        out_ = modifier.addOutput<GenericVectorMessage, double>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        TransformMessage::ConstPtr t_msg = msg::getMessage<TransformMessage>(in_);
        std::shared_ptr<std::vector<double>> out(new std::vector<double>);
        out->resize(6);
        double r, p, y;
        tf::Quaternion q = t_msg->value.getRotation();
        tf::Matrix3x3(q).getRPY(r, p, y);
        tf::Vector3 vec = t_msg->value.getOrigin();
        (*out)[0] = r;
        (*out)[1] = p;
        (*out)[2] = y;
        (*out)[3] = vec.getX();
        (*out)[4] = vec.getY();
        (*out)[5] = vec.getZ();
        msg::publish<GenericVectorMessage, double>(out_, out);
    }

private:
    Input* in_;
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::TransformToVectorDouble, csapex::Node)
