/// HEADER
#include "image_to_point_cloud.h"

/// PROJECT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <type_traits>

CSAPEX_REGISTER_CLASS(csapex::ImageToPointCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ImageToPointCloud::ImageToPointCloud()
{
}

void ImageToPointCloud::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareText("frame", "/camera"));

    std::map<std::string, DepthType> depth_types = {
        {"Direct (meters)", DepthType::METERS},
        {"Direct (millimeters)", DepthType::MILLIMETERS},
        {"Kinect (tangential)" , DepthType::KINECT_TAN}
    };
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet("depth/type",
                                                                                 param::ParameterDescription("Convertion of raw depth values"),
                                                                                 depth_types,
                                                                                 DepthType::METERS),
                            depth_type);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("fov/h", 30.0, 180.0, 90.0, 0.1));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("fov/v", 30.0, 180.0, 90.0, 0.1));

    parameters.addParameter(csapex::param::ParameterFactory::declareInterval("intensity", 0, 255, 0, 255, 1));
}

void ImageToPointCloud::setup(NodeModifier& node_modifier)
{
    input_depth_ = node_modifier.addInput<CvMatMessage>("Depth");
    input_intensity_ = node_modifier.addOptionalInput<CvMatMessage>("Intensity");

    output_ = node_modifier.addOutput<PointCloudMessage>("PointCloud");
}

namespace
{
template <typename PointT>
void setIntensity(PointT&, int)
{

}

template <>
void setIntensity<pcl::PointXYZI>(pcl::PointXYZI& pt, int v)
{
    pt.intensity = v;
}


template <typename PointT, ImageToPointCloud::IntensityType>
void setColor(PointT&, const cv::Vec3b& color)
{

}

template <>
void setColor<pcl::PointXYZRGB, ImageToPointCloud::IntensityType::BGR>(pcl::PointXYZRGB& pt, const cv::Vec3b& color)
{
    pt.r = color.val[2];
    pt.g = color.val[1];
    pt.b = color.val[0];
}
template <>
void setColor<pcl::PointXYZRGB, ImageToPointCloud::IntensityType::RGB>(pcl::PointXYZRGB& pt, const cv::Vec3b& color)
{
    pt.r = color.val[0];
    pt.g = color.val[1];
    pt.b = color.val[2];
}

}

double ImageToPointCloud::depth_to_range(double v) const
{
    switch(depth_type)
    {
    case DepthType::METERS:
        return v;
        break;
    case DepthType::MILLIMETERS:
        return v / 1000.0;
        break;
    case DepthType::KINECT_TAN:
        // https://stackoverflow.com/questions/8824743/kinect-raw-depth-to-distance-in-meters
        return 0.1236 * std::tan(v / 2842.5 + 1.1863);
    default:
        return std::numeric_limits<double>::quiet_NaN();
    }
}

template <typename PointT, typename ImageType, ImageToPointCloud::IntensityType IT>
PointCloudMessage::Ptr ImageToPointCloud::transformImpl(const cv::Mat& depth, const cv::Mat& intensity, std::uint64_t stamp)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    double fov_h = readParameter<double>("fov/h") / 180.0 * M_PI;
    double fov_v = readParameter<double>("fov/v") / 180.0 * M_PI;

    double fov_h2 = fov_h / 2.0;
    double fov_v2 = fov_v / 2.0;

    double w = depth.cols;
    double h = depth.rows;

    double mid_x = w / 2.0;
    double mid_y = h / 2.0;

    std::pair<int,int> range = readParameter<std::pair<int,int> >("intensity");

    for(int y = 0; y < depth.rows; ++y) {
        for(int x = 0; x < depth.cols; ++x) {
            PointT pt;

            double r = depth_to_range(depth.at<ImageType>(y,x));

            if(std::is_same<PointT, pcl::PointXYZI>()){
                int i = intensity.at<uint8_t>(y, x);
                setIntensity(pt, i);

                if(i < range.first || i > range.second) {
                    continue;
                }

            } else if(std::is_same<PointT, pcl::PointXYZRGB>()){
                cv::Vec3b color = intensity.at<cv::Vec3b>(cv::Point(x, y));
                setColor<PointT, IT>(pt, color);
            }

            double angle_x = (x - mid_x) / w * fov_h2;
            double angle_y = (y - mid_y) / h * fov_v2;

            double cx = std::cos(angle_x);
            double sx = std::sin(angle_x);
            double cy = std::cos(angle_y);
            double sy = std::sin(angle_y);

            PointT pt_rot_x;
            pt_rot_x.x = (cx * r);
            pt_rot_x.y = (sx * r);
            pt_rot_x.z = 0;
            pt.x = cy * pt_rot_x.x + sy * pt_rot_x.z;
            pt.y = pt_rot_x.y;
            pt.z = -sy * pt_rot_x.x + cy * pt_rot_x.z;
            cloud->points.push_back(pt);
        }
    }

    PointCloudMessage::Ptr result(new PointCloudMessage(readParameter<std::string>("frame"), stamp));
    result->value = cloud;

    return result;
}

template <typename PointT, ImageToPointCloud::IntensityType IT>
PointCloudMessage::Ptr ImageToPointCloud::transform(const cv::Mat& depth, const cv::Mat& intensity, std::uint64_t stamp)
{
    apex_assert_msg(depth.type() == CV_32FC1 || depth.type() == CV_16UC1,
                    "Depth is not of type CV_32FC1 or CV_16UC1");

    switch(depth.type())
    {
    case CV_32FC1:
        return transformImpl<PointT, float, IT>(depth, intensity, stamp);
    case CV_16UC1:
        return transformImpl<PointT, std::uint16_t, IT>(depth, intensity, stamp);
    default:
        throw std::runtime_error("Unsupported image type");
    }
}

void ImageToPointCloud::process()
{
    CvMatMessage::ConstPtr depth_msg(msg::getMessage<CvMatMessage>(input_depth_));

    PointCloudMessage::Ptr result(new PointCloudMessage(readParameter<std::string>("frame"), depth_msg->stamp_micro_seconds));
    if(msg::hasMessage(input_intensity_)) {
        CvMatMessage::ConstPtr intensity_msg(msg::getMessage<CvMatMessage>(input_intensity_));

        if(intensity_msg->value.type() == CV_8UC1) {
            result = transform<pcl::PointXYZI, IntensityType::INTENSITY>(depth_msg->value, intensity_msg->value, depth_msg->stamp_micro_seconds);

        } else if(intensity_msg->value.type() == CV_8UC3) {
            if(intensity_msg->getEncoding().matches(enc::rgb)) {
                result = transform<pcl::PointXYZRGB, IntensityType::RGB>(depth_msg->value, intensity_msg->value, depth_msg->stamp_micro_seconds);
            } else {
                result = transform<pcl::PointXYZRGB, IntensityType::BGR>(depth_msg->value, intensity_msg->value, depth_msg->stamp_micro_seconds);
            }

        } else {
            node_modifier_->setWarning("Cannot interpret intensity. Please provide either CV_8UC1 or CV_8UC3 images.");
            result = transform<pcl::PointXYZ, IntensityType::NONE>(depth_msg->value, cv::Mat(), depth_msg->stamp_micro_seconds);
        }

    } else {
        result = transform<pcl::PointXYZ, IntensityType::NONE>(depth_msg->value, cv::Mat(), depth_msg->stamp_micro_seconds);
    }


    msg::publish(output_, result);
}
