
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_transform/transform_message.h>
#include <csapex_core_plugins/timestamp_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class VectorDoubleToTransform : public Node
{
public:
    VectorDoubleToTransform()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, double>("Input");
        in_ts_ = modifier.addOptionalInput<TimestampMessage>("Time");
        out_ = modifier.addOutput<TransformMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(csapex::param::ParameterFactory::declareText("frame", "/base_link"), frame);
        params.addParameter(csapex::param::ParameterFactory::declareText("child_frame", "/marlin"), child_frame);
    }

    void process() override
    {
        std::shared_ptr<std::vector<double> const> in = msg::getMessage<GenericVectorMessage, double>(in_);
        TransformMessage::Ptr out(new TransformMessage);
        out->stamp_micro_seconds = 0;

        if(msg::hasMessage(in_ts_)){
            TimestampMessage::ConstPtr stamp = msg::getMessage<TimestampMessage>(in_ts_);
            out->stamp_micro_seconds = std::chrono::duration_cast<std::chrono::microseconds>(stamp->value.time_since_epoch()).count();
        }
        out->frame_id = frame;
        out->child_frame = child_frame;

        tf::Quaternion q;
        q.setRPY(in->at(0), in->at(1), in->at(2));
        tf::Vector3 vec(in->at(3), in->at(4), in->at(5));
        out->value.setRotation(q);
        out->value.setOrigin(vec);

        msg::publish(out_, out);
    }

private:
    Input* in_;
    Input* in_ts_;
    Output* out_;
    std::string frame;
    std::string child_frame;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::VectorDoubleToTransform, csapex::Node)

