/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>
#include <csapex/factory/generic_node_factory.hpp>
#include <csapex/factory/node_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QMetaType>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <compressed_image_transport/compressed_subscriber.h>

CSAPEX_REGISTER_CLASS(csapex::RegisterPlugin, csapex::CorePlugin)

using namespace csapex;

RegisterPlugin::RegisterPlugin()
{
}

struct Image2CvMat
{
    static connection_types::CvMatMessage::Ptr ros2apex(const sensor_msgs::Image::ConstPtr &ros_msg) {
        u_int64_t stamp_micro_seconds = ros_msg->header.stamp.toNSec() * 1e-3;

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
        } else if(source_encoding == sensor_msgs::image_encodings::YUV422) {
            target_encoding = enc::yuv;
        } else if(source_encoding.size() > 6 && source_encoding.substr(6) == "bayer_") {
            target_encoding = enc::mono;
        } else if(source_encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            target_encoding = enc::depth;
        } else {
            target_encoding = enc::unknown;
        }

        connection_types::CvMatMessage::Ptr out(new connection_types::CvMatMessage(target_encoding, stamp_micro_seconds));

        try {
            cv_bridge::toCvShare(ros_msg, ros_msg->encoding)->image.copyTo(out->value);
            out->frame_id = ros_msg->header.frame_id;
        } catch (const cv_bridge::Exception& e) {
            std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "some other exception in cv_bridge: " << e.what() << std::endl;
        }
        return out;
    }
    static sensor_msgs::Image::Ptr apex2ros(const connection_types::CvMatMessage::ConstPtr& apex_msg) {
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
            } else if(apex_msg->getEncoding().matches(enc::mono)) {
                cvb.encoding = sensor_msgs::image_encodings::MONO8;
            } else if(apex_msg->getEncoding().matches(enc::depth)) {
                cvb.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            } else {
                std::cerr << "cannot convert encoding " << apex_msg->getEncoding().toString() << " to ROS" << std::endl;
            }
            break;
        }

        cvb.header.frame_id = apex_msg->frame_id;
        if(apex_msg->stamp_micro_seconds != 0) {
            cvb.header.stamp = cvb.header.stamp.fromNSec(apex_msg->stamp_micro_seconds);
        } else {
            cvb.header.stamp = ros::Time::now();
        }
        return cvb.toImageMsg();
    }
};

struct CompressedImage2CvMat
{
    static connection_types::CvMatMessage::Ptr ros2apex(const sensor_msgs::CompressedImage::ConstPtr &ros_msg) {
        u_int64_t stamp_micro_seconds = ros_msg->header.stamp.toNSec() * 1e-3;

        std::string source_encoding = ros_msg->format;
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
        } else if(source_encoding == sensor_msgs::image_encodings::YUV422) {
            target_encoding = enc::yuv;
        } else if(source_encoding.size() > 6 && source_encoding.substr(6) == "bayer_") {
            target_encoding = enc::mono;
        } else if(source_encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            target_encoding = enc::depth;
        } else {
            target_encoding = enc::unknown;
        }

        connection_types::CvMatMessage::Ptr out(new connection_types::CvMatMessage(target_encoding, stamp_micro_seconds));

        out->value = cv::imdecode(cv::Mat(ros_msg->data),1);

        return out;
    }
    static sensor_msgs::CompressedImage::Ptr apex2ros(const connection_types::CvMatMessage::ConstPtr& apex_msg) {
        // TODO: implement after SBC...
        return nullptr;
    }
};

void testWrap(const connection_types::CvMatMessage& input, int flipcode,
              connection_types::CvMatMessage& output)
{
    cv::flip(input.value, output.value, flipcode);
}

void testWrapDirect(const cv::Mat& input, int flipcode,
                    cv::Mat& output)
{
    cv::flip(input, output, flipcode);
}

struct ParameterInfoTestWrap
{
    static std::string getName(int index) {
        switch(index) {
        case 0:
            return "Image";
        case 1:
            return "flip code";
        case 2:
            return "Flipped Image";
        default:
            return "";
        }
    }
    template <typename P>
    static param::ParameterPtr declareParameter(int index) {
        switch(index) {
        case 1:
            return param::ParameterFactory::declareRange<P>(getName(index), -1, 3, 0, 1);
        default:
            return nullptr;
        }
    }
};

void RegisterPlugin::init(CsApexCore& core)
{
    RosMessageConversion::registerConversion<sensor_msgs::Image, connection_types::CvMatMessage, Image2CvMat>();
    RosMessageConversion::registerConversion<sensor_msgs::CompressedImage, connection_types::CvMatMessage, CompressedImage2CvMat>();

    auto cWrap = GenericNodeFactory::createConstructorFromFunction<ParameterInfoTestWrap>
            (testWrap, "TestWrap");
    cWrap->setDescription("Test direct wrapping.").setIcon(":/combiner.png").setTags({"Wrap", "Test", "Flip"});
    core.getNodeFactory().registerNodeType(cWrap);

    auto cWrapDirect = GenericNodeFactory::createConstructorFromFunction<ParameterInfoTestWrap>
            (testWrapDirect, "TestWrapDirect");
    cWrapDirect->setDescription( "Test directly wrapping a non-apex function.").setIcon(":/combiner.png").setTags({"Wrap", "Test", "Flip"});
    core.getNodeFactory().registerNodeType(cWrapDirect);

    auto cWrapDirectNoInfo = GenericNodeFactory::createConstructorFromFunction
            (testWrapDirect, "TestWrapDirectNoInfo");
    cWrapDirectNoInfo->setDescription("Test directly wrapping a non-apex function with default info.");
    core.getNodeFactory().registerNodeType(cWrapDirectNoInfo);

}
