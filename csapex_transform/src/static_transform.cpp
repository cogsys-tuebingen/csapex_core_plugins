/// HEADER
#include "static_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_math/param/factory.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <tf/transform_datatypes.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

CSAPEX_REGISTER_CLASS(csapex::StaticTransform, csapex::Node)

using namespace csapex;

StaticTransform::StaticTransform()
{
}

void StaticTransform::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareText("frame", "/base_link"), frame);
    parameters.addParameter(csapex::param::factory::declareText("child_frame", "/marlin"), child_frame);

    parameters.addParameter(csapex::param::factory::declareAngle("roll", 0.0), roll);
    parameters.addParameter(csapex::param::factory::declareAngle("pitch", 0.0), pitch);
    parameters.addParameter(csapex::param::factory::declareAngle("yaw", 0.0), yaw);
    parameters.addParameter(csapex::param::factory::declareValue("dx", 0.0), x);
    parameters.addParameter(csapex::param::factory::declareValue("dy", 0.0), y);
    parameters.addParameter(csapex::param::factory::declareValue("dz", 0.0), z);
}

void StaticTransform::process()
{
    if (msg::hasMessage(input_)) {
        connection_types::TransformMessage::ConstPtr pose = msg::getMessage<connection_types::TransformMessage>(input_);
        tf::Transform trafo = pose->value;

        tf::Matrix3x3 rot_mat(trafo.getRotation());
        rot_mat.getEulerYPR(yaw, pitch, roll);

        x = trafo.getOrigin().x();
        y = trafo.getOrigin().y();
        z = trafo.getOrigin().z();

        setParameter("frame", pose->frame_id);
        setParameter("child_frame", pose->child_frame);

        setParameter("roll", roll);
        setParameter("pitch", pitch);
        setParameter("yaw", yaw);
        setParameter("dx", x);
        setParameter("dy", y);
        setParameter("dz", z);
    }

    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
    msg->frame_id = frame;
    msg->child_frame = child_frame;
    msg::publish(output_, msg);
}

void StaticTransform::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addOptionalInput<connection_types::TransformMessage>("Transformation (opt.)");
    output_ = node_modifier.addOutput<connection_types::TransformMessage>("Transformation");
}
