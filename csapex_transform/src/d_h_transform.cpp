
/// COMPONENT
#include <csapex_transform/transform_message.h>
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_math/param/factory.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class DHTransform : public Node
{
public:
    DHTransform()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<double>("variable");
        out_ = modifier.addOutput<TransformMessage>("transform");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        std::map<std::string, int> joint_type = {{"revolute" ,0},
                                                 {"prismatic",1}};

        params.addParameter(param::factory::declareText("frame_id", "link_1"),
                            frame_id_);

        params.addParameter(param::factory::declareText("child_frame", "link_2"),
                            child_frame_);

        params.addParameter(param::factory::declareParameterSet("type",joint_type,0),
                            type_);

        params.addConditionalParameter(param::factory::declareAngle("theta",0.0),
                                       [this](){
            return type_ == 1;
        },
        theta_);

        params.addConditionalParameter(param::factory::declareAngle("trans_z",0.0),
                                       [this](){
            return type_ == 0;
        },
        trans_z_);


        params.addParameter(csapex::param::factory::declareAngle("alpha", 0.0),
                            alpha_);

        params.addParameter(csapex::param::factory::declareValue("trans_x",0.0),
                            trans_x_);

    }

    void process() override
    {
        double variable = msg::getValue<double>(in_);
        TransformMessage::Ptr a_mat(new TransformMessage);
        a_mat->frame_id = frame_id_;
        a_mat->child_frame = child_frame_;

        tf::Transform tx ,tz;
        tx.setIdentity();
        tz.setIdentity();
        tx.setOrigin(tf::Vector3(trans_x_,0,0));
        tf::Transform rx ,rz;
        rx.setIdentity();
        rz.setIdentity();
        tf::Quaternion q;
        q.setRPY(alpha_,0,0);
        rx.setRotation(q);
        if(type_ == 0){
            tz.setOrigin(tf::Vector3(0,0,trans_z_));
            tf::Quaternion qz;
            qz.setRPY(0,0,variable);
            rz.setRotation(qz);
        } else {
            tz.setOrigin(tf::Vector3(0,0,variable));
            tf::Quaternion qz;
            qz.setRPY(0,0,theta_);
            rz.setRotation(qz);
        }
        a_mat->value = rz * tz * tx * rx;
        msg::publish(out_, a_mat);
    }

private:
    Input* in_;
    Output* out_;
    int type_;
    std::string frame_id_;
    std::string child_frame_;
    double theta_;
    double trans_z_;
    double alpha_;
    double trans_x_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::DHTransform, csapex::Node)

