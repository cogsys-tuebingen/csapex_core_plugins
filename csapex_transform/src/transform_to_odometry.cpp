/// HEADER
#include "transform_to_odometry.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

CSAPEX_REGISTER_CLASS(csapex::TransformToOdometry, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


TransformToOdometry::TransformToOdometry()
{
}

void TransformToOdometry::setupParameters()
{
}

void TransformToOdometry::setup()
{
    in_ = modifier_->addInput<connection_types::TransformMessage>("TF");
    out_ = modifier_->addOutput<nav_msgs::Odometry>("Odometry");
}

void TransformToOdometry::process()
{
    connection_types::TransformMessage::Ptr a = in_->getMessage<connection_types::TransformMessage>();

    nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry);
    msg->header.frame_id = a->frame_id;
    msg->header.stamp = msg->header.stamp.fromNSec(a->stamp);
    tf::poseTFToMsg(a->value, msg->pose.pose);

    out_->publish<nav_msgs::Odometry>(msg);
}

