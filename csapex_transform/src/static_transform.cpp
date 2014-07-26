/// HEADER
#include "static_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <tf/transform_datatypes.h>

CSAPEX_REGISTER_CLASS(csapex::StaticTransform, csapex::Node)

using namespace csapex;

StaticTransform::StaticTransform()
{
    double p = 3.2;
    double d = 5.0;

    addParameter(param::ParameterFactory::declare("roll", -p, p, 0.0, 0.001));
    addParameter(param::ParameterFactory::declare("pitch", -p, p, 0.0, 0.001));
    addParameter(param::ParameterFactory::declare("yaw", -p, p, 0.0, 0.001));
    addParameter(param::ParameterFactory::declare("dx", -d, d, 0.0, 0.01));
    addParameter(param::ParameterFactory::declare("dy", -d, d, 0.0, 0.01));
    addParameter(param::ParameterFactory::declare("dz", -d, d, 0.0, 0.01));
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
    output_->publish(msg);
}


void StaticTransform::setup()
{
    output_ = modifier_->addOutput<connection_types::TransformMessage>("Transformation");
}
