#ifndef ROS_CONVERTERS_H
#define ROS_CONVERTERS_H

/// COMPONENT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex_opencv/roi.h>

/// SYSTEM
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/RegionOfInterest.h>

namespace csapex
{

struct Image2CvMat
{
    static connection_types::CvMatMessage::Ptr ros2apex(const sensor_msgs::Image::ConstPtr &ros_msg);
    static sensor_msgs::Image::Ptr apex2ros(const connection_types::CvMatMessage::ConstPtr& apex_msg);
};

struct CompressedImage2CvMat
{
    static connection_types::CvMatMessage::Ptr ros2apex(const sensor_msgs::CompressedImage::ConstPtr &ros_msg);
    static sensor_msgs::CompressedImage::Ptr apex2ros(const connection_types::CvMatMessage::ConstPtr& apex_msg);
};

struct ConvertROI
{
    static void ros2apex(const sensor_msgs::RegionOfInterest::ConstPtr &ros_msg, csapex::Roi& out);
    static connection_types::RoiMessage::Ptr ros2apex(const sensor_msgs::RegionOfInterest::ConstPtr &ros_msg);
    static sensor_msgs::RegionOfInterest::Ptr apex2ros(const connection_types::RoiMessage::ConstPtr& apex_msg);
    static sensor_msgs::RegionOfInterest::Ptr apex2ros(const csapex::Roi& apex_roi);
};

}

#endif // ROS_CONVERTERS_H
