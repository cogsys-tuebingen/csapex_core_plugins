/// HEADER
#include "static_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/range_parameter.h>
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
    parameters.addParameter(csapex::param::ParameterFactory::declareText("frame", "/base_link"), frame);
    parameters.addParameter(csapex::param::ParameterFactory::declareText("child_frame", "/marlin"), child_frame);

    parameters.addParameter(csapex::param::ParameterFactory::declareAngle("roll", 0.0), roll);
    parameters.addParameter(csapex::param::ParameterFactory::declareAngle("pitch", 0.0), pitch);
    parameters.addParameter(csapex::param::ParameterFactory::declareAngle("yaw", 0.0), yaw);
    parameters.addParameter(csapex::param::ParameterFactory::declareValue("dx", 0.0), x);
    parameters.addParameter(csapex::param::ParameterFactory::declareValue("dy", 0.0), y);
    parameters.addParameter(csapex::param::ParameterFactory::declareValue("dz", 0.0), z);
}

void StaticTransform::process()
{

}

void StaticTransform::tick()
{
    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
    msg->frame_id = frame;
    msg->child_frame = child_frame;
    msg::publish(output_, msg);
}


void StaticTransform::setup(NodeModifier& node_modifier)
{
    output_ = node_modifier.addOutput<connection_types::TransformMessage>("Transformation");
}
