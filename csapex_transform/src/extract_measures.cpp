/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

using namespace csapex::connection_types;


namespace csapex
{

class ExtractMeasures : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<TransformMessage>("Transform");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareAngle("roll", 0));
        params.addParameter(param::ParameterFactory::declareAngle("pitch", 0));
        params.addParameter(param::ParameterFactory::declareBool("yaw/cap", true), cap_yaw_);
        params.addParameter(param::ParameterFactory::declareAngle("yaw", 0));

        params.addParameter(param::ParameterFactory::declareValue("dx", 0.));
        params.addParameter(param::ParameterFactory::declareValue("dy", 0.));
        params.addParameter(param::ParameterFactory::declareValue("dz", 0.));

        params.addParameter(param::ParameterFactory::declareValue("norm", 0.));
    }

    void process()
    {
        TransformMessage::ConstPtr trafo = msg::getMessage<TransformMessage>(in_);
        tf::Transform tf_trafo = trafo->value;

        tf::Matrix3x3 rotation(tf_trafo.getRotation());
        double roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);

        if(cap_yaw_) {
            yaw = cap(yaw);
        }

        setParameter("roll", roll);
        setParameter("pitch", pitch);
        setParameter("yaw", yaw);


        tf::Vector3 delta = tf_trafo.getOrigin();
        setParameter("dx", delta.x());
        setParameter("dy", delta.y());
        setParameter("dz", delta.z());

        setParameter("norm", delta.length());
    }

    static double cap(double a)
    {
        double r = a;
        while(r < -M_PI) r += 2*M_PI;
        while(r >= M_PI) r -= 2*M_PI;
        if(r >= M_PI / 2.0) {
            return r - M_PI;
        } else if(r <= -M_PI / 2.0) {
            return r + M_PI;
        } else {
            return r;
        }
    }

private:
    Input* in_;

    bool cap_yaw_;
};

}

CSAPEX_REGISTER_CLASS(csapex::ExtractMeasures, csapex::Node)

