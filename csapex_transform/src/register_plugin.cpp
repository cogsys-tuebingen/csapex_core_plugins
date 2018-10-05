/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_ros/tf_listener.h>
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <tf2_msgs/TFMessage.h>

CSAPEX_REGISTER_CLASS(csapex::RegisterTransformPlugin, csapex::CorePlugin)

using namespace csapex;

struct ConvertTf
{
    static typename connection_types::TransformMessage::Ptr ros2apex(const typename tf2_msgs::TFMessage::ConstPtr& ros_msg)
    {
        typename connection_types::TransformMessage::Ptr out(new connection_types::TransformMessage);
        const geometry_msgs::TransformStamped& tf(ros_msg->transforms.front());

        tf::transformMsgToTF(tf.transform, out->value);
        out->child_frame = tf.child_frame_id;
        out->frame_id = tf.header.frame_id;
        out->stamp_micro_seconds = tf.header.stamp.toNSec();

        return out;
    }
    static typename tf2_msgs::TFMessage::Ptr apex2ros(const typename connection_types::TransformMessage::ConstPtr& apex_msg)
    {
        typename tf2_msgs::TFMessage::Ptr out(new tf2_msgs::TFMessage);
        geometry_msgs::TransformStamped tf;

        tf::transformTFToMsg(apex_msg->value, tf.transform);
        tf.child_frame_id = apex_msg->child_frame;
        tf.header.frame_id = apex_msg->frame_id;
        tf.header.stamp = tf.header.stamp.fromNSec(apex_msg->stamp_micro_seconds * 1e3);

        out->transforms.push_back(tf);
        return out;
    }
};

RegisterTransformPlugin::RegisterTransformPlugin()
{
}

void RegisterTransformPlugin::prepare(Settings&)
{
    Tag::createIfNotExists("Transform");
    Tag::createIfNotExists("Time");

    auto init = std::bind(&TFListener::start);

    if (ROSHandler::instance().isConnected()) {
        init();
    } else {
        observe(ROSHandler::instance().connected, init);
    }

    observe(ROSHandler::instance().shutdown, std::bind(&TFListener::stop));

    RosMessageConversion::registerConversion<tf2_msgs::TFMessage, connection_types::TransformMessage, ConvertTf>();
}
