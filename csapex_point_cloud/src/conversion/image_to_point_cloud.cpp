/// HEADER
#include "image_to_point_cloud.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_ros/time_stamp_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <boost/type_traits.hpp>

CSAPEX_REGISTER_CLASS(csapex::ImageToPointCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ImageToPointCloud::ImageToPointCloud()
{
    addParameter(param::ParameterFactory::declareText("frame", "/camera"));

    addParameter(param::ParameterFactory::declareRange("fov/h", 30.0, 180.0, 90.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("fov/v", 30.0, 180.0, 90.0, 0.1));

    addParameter(param::ParameterFactory::declareInterval("intensity", 0, 255, 0, 255, 1));

    addParameter(param::ParameterFactory::declareBool("experimental error compensation", true));

    addParameter(param::ParameterFactory::declareRange("correct/start", 0, 255, 255, 1));
    addParameter(param::ParameterFactory::declareRange("correct/f", -0.2, 0.2, 0.0, 0.01));
}

void ImageToPointCloud::setup()
{
    input_depth_ = modifier_->addInput<CvMatMessage>("Depth");
    input_intensity_ = modifier_->addOptionalInput<CvMatMessage>("Intensity");

    output_ = modifier_->addOutput<PointCloudMessage>("PointCloud");
}

namespace
{
template <typename PointT>
void set(PointT&, int)
{

}

template <>
void set<pcl::PointXYZI>(pcl::PointXYZI& pt, int v)
{
    pt.intensity = v;
}
}

template <typename PointT>
PointCloudMessage::Ptr ImageToPointCloud::transform(const cv::Mat& depth, const cv::Mat& intensity)
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

    bool error_comp = readParameter<bool>("experimental error compensation");
    int correction_start = readParameter<int>("correct/start");
    double correction_f = readParameter<double>("correct/f");

    if(boost::is_same<PointT, pcl::PointXYZI>()) {
        apex_assert_hard(intensity.type() == CV_8UC1);
    }
    apex_assert_hard(depth.type() == CV_32FC1);

    std::pair<int,int> range = readParameter<std::pair<int,int> >("intensity");

    for(int y = 0; y < depth.rows; ++y) {
        for(int x = 0; x < depth.cols; ++x) {
            PointT pt;

            double r = depth.at<float>(y,x);

            if(boost::is_same<PointT, pcl::PointXYZI>()){
                int i = intensity.at<uint8_t>(y, x);
                set<PointT>(pt, i);

                if(i < range.first || i > range.second) {
                    continue;
                }

                if(error_comp && i < correction_start) {
                    double f = (i-correction_start) / (double) (255 - correction_start);

                    r += f * correction_f;
                }
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

    // TODO: get stamp from depth image
    PointCloudMessage::Ptr result(new PointCloudMessage(readParameter<std::string>("frame"), 0));
    result->value = cloud;

    return result;
}

void ImageToPointCloud::process()
{
    CvMatMessage::ConstPtr depth_msg(input_depth_->getMessage<CvMatMessage>());

    PointCloudMessage::Ptr result(new PointCloudMessage(readParameter<std::string>("frame"), depth_msg->stamp));
    if(input_intensity_->hasMessage()) {
        CvMatMessage::ConstPtr intensity_msg(input_intensity_->getMessage<CvMatMessage>());
        result = transform<pcl::PointXYZI>(depth_msg->value, intensity_msg->value);

    } else {
        result = transform<pcl::PointXYZ>(depth_msg->value, cv::Mat());
    }


    output_->publish(result);
}
