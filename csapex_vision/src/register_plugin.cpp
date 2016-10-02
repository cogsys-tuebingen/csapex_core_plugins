/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_opencv/cv_mat_message.h>

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

namespace impl
{
void convertImage(const connection_types::CvMatMessage::ConstPtr& in, cv_bridge::CvImage& out)
{
    out.image = in->value;

    switch(in->value.type()) {
    case CV_8UC1:
        out.encoding = sensor_msgs::image_encodings::MONO8;
        break;
    default:
    case CV_8UC3:
        if(in->getEncoding().matches(enc::bgr)) {
            out.encoding = sensor_msgs::image_encodings::BGR8;
        } else if(in->getEncoding().matches(enc::rgb)) {
            out.encoding = sensor_msgs::image_encodings::RGB8;
        } else if(in->getEncoding().matches(enc::yuv)) {
            out.encoding = sensor_msgs::image_encodings::YUV422;
        } else if(in->getEncoding().matches(enc::mono)) {
            out.encoding = sensor_msgs::image_encodings::MONO8;
        } else if(in->getEncoding().matches(enc::depth)) {
            out.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        } else {
            std::cerr << "cannot convert encoding " << in->getEncoding().toString() << " to ROS" << std::endl;
        }
        break;
    }

    out.header.frame_id = in->frame_id;
    if(in->stamp_micro_seconds != 0) {
        out.header.stamp = out.header.stamp.fromNSec(in->stamp_micro_seconds);
    } else {
        out.header.stamp = ros::Time::now();
    }
}

csapex::Encoding convertEncoding(const std::string& in)
{
    if(in == sensor_msgs::image_encodings::BGR8 || in == sensor_msgs::image_encodings::BGR16) {
        return enc::bgr;
    } else if(in == sensor_msgs::image_encodings::RGB8 || in == sensor_msgs::image_encodings::RGB16) {
        return enc::rgb;
    } else if(in == sensor_msgs::image_encodings::MONO8 || in == sensor_msgs::image_encodings::MONO16) {
        return enc::mono;
    } else if(in == sensor_msgs::image_encodings::YUV422) {
        return enc::yuv;
    } else if(in.size() > 6 && in.substr(6) == "bayer_") {
        return enc::mono;
    } else if(in == sensor_msgs::image_encodings::TYPE_16UC1) {
        return enc::depth;
    } else {
        return enc::unknown;
    }
}
}

struct Image2CvMat
{
    static connection_types::CvMatMessage::Ptr ros2apex(const sensor_msgs::Image::ConstPtr &ros_msg) {
        u_int64_t stamp_micro_seconds = ros_msg->header.stamp.toNSec() * 1e-3;

        csapex::Encoding target_encoding = impl::convertEncoding(ros_msg->encoding);

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
        impl::convertImage(apex_msg, cvb);
        return cvb.toImageMsg();
    }
};

struct CompressedImage2CvMat
{
    static connection_types::CvMatMessage::Ptr ros2apex(const sensor_msgs::CompressedImage::ConstPtr &ros_msg) {
        u_int64_t stamp_micro_seconds = ros_msg->header.stamp.toNSec() * 1e-3;

        connection_types::CvMatMessage::Ptr out;

        try {
            cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(ros_msg);
            csapex::Encoding target_encoding = impl::convertEncoding(img->encoding);

            out.reset(new connection_types::CvMatMessage(target_encoding, stamp_micro_seconds));
            out->value = img->image;
            out->frame_id = ros_msg->header.frame_id;

        } catch (const cv_bridge::Exception& e) {
            std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "some other exception in cv_bridge: " << e.what() << std::endl;
        }

        return out;
    }
    static sensor_msgs::CompressedImage::Ptr apex2ros(const connection_types::CvMatMessage::ConstPtr& apex_msg) {
        cv_bridge::CvImage cvb;
        impl::convertImage(apex_msg, cvb);
        return cvb.toCompressedImageMsg();
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
    static csapex::param::ParameterPtr declareParameter(int index) {
        switch(index) {
        case 1:
            return csapex::param::ParameterFactory::declareRange<P>(getName(index), -1, 3, 0, 1);
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
