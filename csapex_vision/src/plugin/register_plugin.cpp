/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>
#include <csapex/model/connector_out.h>

/// SYSTEM
#include <boost/bind.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <QMetaType>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

CSAPEX_REGISTER_CLASS(csapex::RegisterPlugin, csapex::CorePlugin)

using namespace csapex;

RegisterPlugin::RegisterPlugin()
{
}

struct Image2CvMat
{
    static connection_types::CvMatMessage::Ptr ros2apex(const sensor_msgs::Image::ConstPtr &ros_msg) {
        connection_types::CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::bgr)); // TODO: use encoding from ROS!
        try {
            cv_bridge::toCvShare(ros_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(out->value);
            out->frame_id = ros_msg->header.frame_id;
        } catch (cv_bridge::Exception& e) {
            std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        }
        return out;
    }
    static sensor_msgs::Image::Ptr apex2ros(const connection_types::CvMatMessage::Ptr& apex_msg) {
        cv_bridge::CvImage cvb;
        cvb.image = apex_msg->value;

        switch(apex_msg->value.type()) {
        case CV_8UC1:
            cvb.encoding = sensor_msgs::image_encodings::MONO8;
            break;
        default:
        case CV_8UC3:
            cvb.encoding = sensor_msgs::image_encodings::BGR8;
            break;
        }

        cvb.header.frame_id = apex_msg->frame_id;
        cvb.header.stamp = ros::Time::now();
        return cvb.toImageMsg();
    }
};

void RegisterPlugin::init(CsApexCore& core)
{
    Tag::createIfNotExists("Vision");
    Tag::createIfNotExists("Filter");
    Tag::createIfNotExists("Image Combiner");
    Tag::createIfNotExists("ROI");
    Tag::createIfNotExists("Pattern Recognition");

    qRegisterMetaType<cv::Mat>("cv::Mat");

    MessageFactory::registerMessage<connection_types::CvMatMessage>();

    RosMessageConversion::registerConversion<sensor_msgs::Image, connection_types::CvMatMessage, Image2CvMat>();

    ConnectionType::setDefaultConnectionType<connection_types::CvMatMessage>();
}
