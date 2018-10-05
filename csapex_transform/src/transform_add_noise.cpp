
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <random>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class TransformAddNoise : public Node
{
public:
    TransformAddNoise() : noise_(6)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<TransformMessage>("Input");
        out_ = modifier.addOutput<TransformMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("std_x", 0.0, 1.0, 0.01, 0.01), [this](param::Parameter* p) {
            double std = p->as<double>();
            noise_[0] = std::normal_distribution<double>(0, std);
        });

        params.addParameter(param::ParameterFactory::declareRange("std_y", 0.0, 1.0, 0.01, 0.01), [this](param::Parameter* p) {
            double std = p->as<double>();
            noise_[1] = std::normal_distribution<double>(0, std);
        });
        params.addParameter(param::ParameterFactory::declareRange("std_z", 0.0, 1.0, 0.01, 0.01), [this](param::Parameter* p) {
            double std = p->as<double>();
            noise_[2] = std::normal_distribution<double>(0, std);
        });
        params.addParameter(param::ParameterFactory::declareRange("std_roll", 0.0, 1.0, 0.01, 0.01), [this](param::Parameter* p) {
            double std = p->as<double>();
            noise_[3] = std::normal_distribution<double>(0, std);
        });
        params.addParameter(param::ParameterFactory::declareRange("std_pitch", 0.0, 1.0, 0.01, 0.01), [this](param::Parameter* p) {
            double std = p->as<double>();
            noise_[4] = std::normal_distribution<double>(0, std);
        });
        params.addParameter(param::ParameterFactory::declareRange("std_yaw", 0.0, 1.0, 0.01, 0.01), [this](param::Parameter* p) {
            double std = p->as<double>();
            noise_[5] = std::normal_distribution<double>(0, std);
        });
    }

    void process() override
    {
        TransformMessage::ConstPtr msg_in = msg::getMessage<TransformMessage>(in_);
        TransformMessage::Ptr msg_out(new TransformMessage);
        msg_out->frame_id = msg_in->frame_id;
        msg_out->child_frame = msg_in->child_frame;
        msg_out->stamp_micro_seconds = msg_in->stamp_micro_seconds;
        tf::Vector3 pos = msg_in->getOrigin();
        double r, p, y;
        msg_in->getBasis().getRPY(r, p, y);

        double dx = noise_[0](generator_);
        double dy = noise_[1](generator_);
        double dz = noise_[2](generator_);
        double droll = noise_[3](generator_);
        double dpitch = noise_[4](generator_);
        double dyaw = noise_[5](generator_);

        pos += tf::Vector3(dx, dy, dz);
        tf::Quaternion q;
        q.setRPY(r + droll, p + dpitch, y + dyaw);
        msg_out->setRotation(q);
        msg_out->setOrigin(pos);
        msg::publish(out_, msg_out);
    }

private:
    Input* in_;
    Output* out_;

    std::vector<std::normal_distribution<double>> noise_;
    std::mt19937 generator_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::TransformAddNoise, csapex::Node)
