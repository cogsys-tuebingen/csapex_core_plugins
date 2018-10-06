/// HEADER
#include "pointcloud_to_pointmatrix.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

/// SYSTEM
#include <pcl/conversions.h>
#include <pcl/point_types.h>

CSAPEX_REGISTER_CLASS(csapex::PointCloudToPointMatrix, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PointCloudToPointMatrix::PointCloudToPointMatrix()
{
}

void PointCloudToPointMatrix::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

    boost::apply_visitor(PointCloudMessage::Dispatch<PointCloudToPointMatrix>(this, msg), msg->value);
}

void PointCloudToPointMatrix::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    output_ = node_modifier.addOutput<CvMatMessage>("Point Matrix");
    mask_ = node_modifier.addOutput<CvMatMessage>("Vadility");
}

namespace implementation
{
template <class PointT>
struct Impl
{
    static void convert(const typename pcl::PointCloud<PointT>::ConstPtr cloud, CvMatMessage::Ptr& matrix, CvMatMessage::Ptr& mask)
    {
        matrix->setEncoding(enc::pointXYZ);

        int height = cloud->height;
        int width = cloud->width;
        cv::Mat& matrix_value = matrix->value;
        cv::Mat& mask_value = mask->value;
        matrix_value = cv::Mat(height, width, CV_32FC3);
        mask_value = cv::Mat(height, width, CV_8UC1, 255);

        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                PointT pos = cloud->at(i * width + j);
                matrix_value.at<float>(i, (j * 3 + 0)) = pos.x;
                matrix_value.at<float>(i, (j * 3 + 1)) = pos.y;
                matrix_value.at<float>(i, (j * 3 + 2)) = pos.z;
                if (pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask_value.at<uchar>(i, j) = 0;
                }
            }
        }
    }
};
template <>
struct Impl<pcl::PointXY>
{
    static void convert(const typename pcl::PointCloud<pcl::PointXY>::ConstPtr cloud, CvMatMessage::Ptr& matrix, CvMatMessage::Ptr& mask)
    {
        std::runtime_error("Conversion is not supported for pcl::PointXY!");
    }
};

template <>
struct Impl<pcl::PointXYZI>
{
    static void convert(const typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, CvMatMessage::Ptr& matrix, CvMatMessage::Ptr& mask)
    {
        matrix->setEncoding(enc::pointXYZI);

        int height = cloud->height;
        int width = cloud->width;
        cv::Mat& matrix_value = matrix->value;
        cv::Mat& mask_value = mask->value;

        matrix_value = cv::Mat(height, width, CV_32FC4);
        mask_value = cv::Mat(height, width, CV_8UC1, 255);

        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                pcl::PointXYZI pos = cloud->at(i * width + j);
                matrix_value.at<float>(i, (j * 4 + 0)) = pos.x;
                matrix_value.at<float>(i, (j * 4 + 1)) = pos.y;
                matrix_value.at<float>(i, (j * 4 + 2)) = pos.z;
                matrix_value.at<float>(i, (j * 4 + 3)) = pos.intensity;
                if (pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask_value.at<uchar>(i, j) = 0;
                }
            }
        }
    }
};

template <>
struct Impl<pcl::PointXYZRGB>
{
    static void convert(const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, CvMatMessage::Ptr& matrix, CvMatMessage::Ptr& mask)
    {
        matrix->setEncoding(enc::pointXYZRGB);
        int height = cloud->height;
        int width = cloud->width;
        cv::Mat& matrix_value = matrix->value;
        cv::Mat& mask_value = mask->value;

        matrix_value = cv::Mat(height, width, CV_32FC(6), cv::Scalar());
        mask_value = cv::Mat(height, width, CV_8UC1, 255);

        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                pcl::PointXYZRGB pos = cloud->at(i * width + j);
                matrix_value.at<float>(i, (j * 6 + 0)) = pos.x;
                matrix_value.at<float>(i, (j * 6 + 1)) = pos.y;
                matrix_value.at<float>(i, (j * 6 + 2)) = pos.z;
                matrix_value.at<float>(i, (j * 6 + 3)) = pos.r;
                matrix_value.at<float>(i, (j * 6 + 4)) = pos.g;
                matrix_value.at<float>(i, (j * 6 + 5)) = pos.b;
                if (pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask_value.at<uchar>(i, j) = 0;
                }
            }
        }
    }
};

}  // namespace implementation

template <class PointT>
void PointCloudToPointMatrix::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    CvMatMessage::Ptr out(new CvMatMessage(enc::unknown, cloud->header.frame_id, cloud->header.stamp));
    CvMatMessage::Ptr mask(new CvMatMessage(enc::mono, cloud->header.frame_id, cloud->header.stamp));
    out->frame_id = cloud->header.frame_id;
    mask->frame_id = cloud->header.frame_id;
    implementation::Impl<PointT>::convert(cloud, out, mask);
    msg::publish(output_, out);
    msg::publish(mask_, mask);
}
