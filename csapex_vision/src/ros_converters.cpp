/// HEADER
#include <csapex_vision/ros_converters.h>

/// COMPONENT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex_vision/ros_converters.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <compressed_image_transport/compressed_subscriber.h>

using namespace csapex;


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
        out.header.stamp = out.header.stamp.fromNSec(in->stamp_micro_seconds * 1e3);
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

connection_types::CvMatMessage::Ptr Image2CvMat::ros2apex(const sensor_msgs::Image::ConstPtr &ros_msg)
{
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


sensor_msgs::Image::Ptr Image2CvMat::apex2ros(const connection_types::CvMatMessage::ConstPtr& apex_msg)
{
    cv_bridge::CvImage cvb;
    impl::convertImage(apex_msg, cvb);
    return cvb.toImageMsg();
}


connection_types::CvMatMessage::Ptr CompressedImage2CvMat::ros2apex(const sensor_msgs::CompressedImage::ConstPtr &ros_msg)
{
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

sensor_msgs::CompressedImage::Ptr CompressedImage2CvMat::apex2ros(const connection_types::CvMatMessage::ConstPtr& apex_msg)
{
    cv_bridge::CvImage cvb;
    impl::convertImage(apex_msg, cvb);
    return cvb.toCompressedImageMsg();
}


void ConvertROI::ros2apex(const sensor_msgs::RegionOfInterest::ConstPtr &ros_msg, Roi &out)
{
    out.setH(ros_msg->height);
    out.setW(ros_msg->width);
    out.setX(ros_msg->x_offset);
    out.setY(ros_msg->y_offset);
}

connection_types::RoiMessage::Ptr ConvertROI::ros2apex(const sensor_msgs::RegionOfInterest::ConstPtr &ros_msg)
{
    connection_types::RoiMessage::Ptr out(new connection_types::RoiMessage);
    out->value = csapex::Roi();

    out->frame_id = "/";
    out->stamp_micro_seconds = 0;

    ros2apex(ros_msg, out->value);

    return out;
}

sensor_msgs::RegionOfInterest::Ptr ConvertROI::apex2ros(const csapex::Roi& apex_roi)
{
    sensor_msgs::RegionOfInterest::Ptr out(new sensor_msgs::RegionOfInterest);

    out->height   = apex_roi.h();
    out->width    = apex_roi.w();
    out->x_offset = apex_roi.x();
    out->y_offset = apex_roi.y();

    return out;
}

sensor_msgs::RegionOfInterest::Ptr ConvertROI::apex2ros(const connection_types::RoiMessage::ConstPtr& apex_msg)
{
    return  apex2ros(apex_msg->value);
}
