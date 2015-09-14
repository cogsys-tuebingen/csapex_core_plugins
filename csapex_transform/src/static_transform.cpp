/// HEADER
#include "static_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>

/// SYSTEM
#include <tf/transform_datatypes.h>

CSAPEX_REGISTER_CLASS(csapex::StaticTransform, csapex::Node)

using namespace csapex;

StaticTransform::StaticTransform()
{
}

void StaticTransform::setupParameters(Parameterizable &parameters)
{
    double p = 3.2;
    double d = 5.0;

    parameters.addParameter(param::ParameterFactory::declareText("frame", "/base_link"));
    parameters.addParameter(param::ParameterFactory::declareText("child_frame", "/marlin"));

    parameters.addParameter(param::ParameterFactory::declareRange("roll", -p, p, 0.0, 0.001));
    parameters.addParameter(param::ParameterFactory::declareRange("pitch", -p, p, 0.0, 0.001));
    parameters.addParameter(param::ParameterFactory::declareRange("yaw", -p, p, 0.0, 0.001));
    parameters.addParameter(param::ParameterFactory::declareRange("dx", -d, d, 0.0, 0.01));
    parameters.addParameter(param::ParameterFactory::declareRange("dy", -d, d, 0.0, 0.01));
    parameters.addParameter(param::ParameterFactory::declareRange("dz", -d, d, 0.0, 0.01));
}

void StaticTransform::process()
{

}

void StaticTransform::tick()
{
    double roll = readParameter<double>("roll");
    double pitch = readParameter<double>("pitch");
    double yaw = readParameter<double>("yaw");
    double x = readParameter<double>("dx");
    double y = readParameter<double>("dy");
    double z = readParameter<double>("dz");

    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
    msg->frame_id = readParameter<std::string>("frame");
    msg->child_frame = readParameter<std::string>("child_frame");
    msg::publish(output_, msg);
}


void StaticTransform::setup(NodeModifier& node_modifier)
{
    output_ = node_modifier.addOutput<connection_types::TransformMessage>("Transformation");
}
