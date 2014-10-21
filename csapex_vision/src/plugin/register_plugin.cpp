/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>
#include <csapex/msg/output.h>
#include <csapex/factory/generic_node_factory.hpp>
#include <csapex/model/node_factory.h>

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
        u_int64_t stamp = ros_msg->header.stamp.toNSec();

        std::string source_encoding = ros_msg->encoding;
        csapex::Encoding target_encoding;
        if(source_encoding == sensor_msgs::image_encodings::BGR8 ||
                source_encoding == sensor_msgs::image_encodings::BGR16) {
            target_encoding = enc::bgr;
        } else if(source_encoding == sensor_msgs::image_encodings::RGB8 ||
                  source_encoding == sensor_msgs::image_encodings::RGB16) {
            target_encoding = enc::rgb;
        } else if(source_encoding == sensor_msgs::image_encodings::MONO8 ||
                  source_encoding == sensor_msgs::image_encodings::MONO16) {
            target_encoding = enc::mono;
        } else {
            std::cerr << "unsupported image encoding: " << source_encoding << std::endl;
        }

        connection_types::CvMatMessage::Ptr out(new connection_types::CvMatMessage(target_encoding, stamp));
        try {
            cv_bridge::toCvShare(ros_msg, ros_msg->encoding)->image.copyTo(out->value);
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
            if(apex_msg->getEncoding().matches(enc::bgr)) {
                cvb.encoding = sensor_msgs::image_encodings::BGR8;
            } else if(apex_msg->getEncoding().matches(enc::rgb)) {
                cvb.encoding = sensor_msgs::image_encodings::RGB8;
            } else if(apex_msg->getEncoding().matches(enc::yuv)) {
                cvb.encoding = sensor_msgs::image_encodings::YUV422;
            }
            break;
        }

        cvb.header.frame_id = apex_msg->frame_id;
        if(apex_msg->stamp != 0) {
            cvb.header.stamp = cvb.header.stamp.fromNSec(apex_msg->stamp);
        } else {
            cvb.header.stamp = ros::Time::now();
        }
        return cvb.toImageMsg();
    }
};

void testWrap(const connection_types::CvMatMessage& input, int flipcode, connection_types::CvMatMessage& output)
{
    cv::flip(input.value, output.value, flipcode);
}

void RegisterPlugin::init(CsApexCore& core)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");

    RosMessageConversion::registerConversion<sensor_msgs::Image, connection_types::CvMatMessage, Image2CvMat>();

    ConnectionType::setDefaultConnectionType(connection_types::makeEmpty<connection_types::CvMatMessage>());

    core.getNodeFactory().registerNodeType(GenericNodeFactory::createConstructorFromFunction(testWrap,
                                                                                             "TestWrap", "Test direct wrapping",
                                                                                             core.getSettings()));
}
