/// HEADER
#include "transform_filter.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING


#define RAD_TO_DEG(x) ((x) * 57.29578)

CSAPEX_REGISTER_CLASS(csapex::TransformFilter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformFilter::TransformFilter()
{
    addTag(Tag::get("Transform"));
    addTag(Tag::get("Filter"));
}

void TransformFilter::setup()
{
    input_transform_ = addInput<connection_types::TransformMessage>("Raw Transformation");

    output_transform_ = addOutput<connection_types::TransformMessage>("Filtered Transformation");
    output_text_ = addOutput<DirectMessage<std::string> >("String"); // create a debug output
}

void TransformFilter::process()
{
    // Output Varibales
    double x,y,z;
    double roll, pitch, yaw;
    std::stringstream stringstream;

    // Get data from input
    TransformMessage::Ptr trafo_msg = input_transform_->getMessage<TransformMessage>();
    tf::Transform tf_raw = trafo_msg->value;


    tfToXYZrpy(tf_raw, x, y, z, roll, pitch, yaw);



    // Publish Output - Debug
    stringstream << "x = " << x << ",y = " << y << ",z = " << z
                 << ",r = " << RAD_TO_DEG(roll) << ",p = " << RAD_TO_DEG(pitch)<< ",y = " << RAD_TO_DEG(yaw);
    DirectMessage<std::string>::Ptr text_msg(new DirectMessage<std::string>);
    text_msg->value = stringstream.str();
    output_text_->publish(text_msg);

    // Publish Output - Transformation
    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
    output_transform_->publish(msg);
}

void TransformFilter::tfToXYZrpy(tf::Transform in, double& x, double& y, double& z, double& roll, double& pitch, double& yaw)
{
    // Extract the origin
    tf::Vector3 origin = in.getOrigin();
    x = origin.getX();
    y = origin.getY();
    z = origin.getZ();

    // Extract the rotation
    tf::Matrix3x3 rotation(in.getRotation());
    rotation.getEulerYPR(roll, pitch, yaw);
}

void TransformFilter::runFilter(tf::Transform in_new, tf::Transform out)
{
    double x,y,z;
    double roll, pitch, yaw;

    tfToXYZrpy(in_new, x, y, z, roll, pitch, yaw);
}
