/// HEADER
#include "pointcloud_to_pointmatrix.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <pcl/point_types.h>
#include <pcl/conversions.h>

CSAPEX_REGISTER_CLASS(csapex::PointCloudToPointMatrix, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PointCloudToPointMatrix::PointCloudToPointMatrix()
{
}

void PointCloudToPointMatrix::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

    boost::apply_visitor (PointCloudMessage::Dispatch<PointCloudToPointMatrix>(this, msg), msg->value);
}

void PointCloudToPointMatrix::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<PointCloudMessage>("PointCloud");
    output_ = node_modifier.addOutput<CvMatMessage>("Point Matrix");
    mask_   = node_modifier.addOutput<CvMatMessage>("Vadility");
}

namespace implementation {
template<class PointT>
struct Impl {
    static void convert(const typename pcl::PointCloud<PointT>::ConstPtr cloud, cv::Mat &matrix, cv::Mat &mask)
    {
        int height = cloud->height;
        int width  = cloud->width;
        matrix = cv::Mat(height, width, CV_32FC3);
        mask   = cv::Mat(height, width, CV_8UC1, 255);

        for(int i = 0 ; i < height ; ++i) {
            for(int j = 0 ; j < width ; ++j) {
                PointT pos = cloud->at(i * width + j);
                matrix.at<float>(i, (j * 3 + 0)) = pos.x;
                matrix.at<float>(i, (j * 3 + 1)) = pos.y;
                matrix.at<float>(i, (j * 3 + 2)) = pos.z;
                if(pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask.at<uchar>(i,j) = 0;
                }
            }
        }
    }
};
template <>
struct Impl<pcl::PointXY> {
    static void convert(const typename pcl::PointCloud<pcl::PointXY>::ConstPtr cloud, cv::Mat &matrix, cv::Mat &mask)
    {
        std::runtime_error("Conversion is not supported for pcl::PointXY!");
    }
};

template<>
struct Impl<pcl::PointXYZI> {
    static void convert(const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, cv::Mat &matrix, cv::Mat &mask)
    {
        int height = cloud->height;
        int width  = cloud->width;
        matrix = cv::Mat(height, width, CV_32FC4);
        mask   = cv::Mat(height, width, CV_8UC1, 255);

        for(int i = 0 ; i < height ; ++i) {
            for(int j = 0 ; j < width ; ++j) {
                pcl::PointXYZI pos = cloud->at(i * width + j);
                matrix.at<float>(i, (j * 4 + 0)) = pos.x;
                matrix.at<float>(i, (j * 4 + 1)) = pos.y;
                matrix.at<float>(i, (j * 4 + 2)) = pos.z;
                matrix.at<float>(i, (j * 4 + 3)) = pos.intensity;
                if(pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask.at<uchar>(i,j) = 0;
                }
            }
        }
    }
};

}


template <class PointT>
void PointCloudToPointMatrix::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    #warning "FIX ENCODING"
    CvMatMessage::Ptr out(new CvMatMessage(enc::unknown, cloud->header.stamp));
    CvMatMessage::Ptr mask(new CvMatMessage(enc::mono, cloud->header.stamp));
    implementation::Impl<PointT>::convert(cloud, out->value, mask->value);
    msg::publish(output_, out);
    msg::publish(mask_, mask);
}
