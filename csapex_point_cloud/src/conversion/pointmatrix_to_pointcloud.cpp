/// HEADER
#include "pointmatrix_to_pointcloud.h"

/// PROJECT
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <pcl/point_types.h>
#include <pcl/conversions.h>

CSAPEX_REGISTER_CLASS(csapex::PointmatrixToPointcloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace implementation {
inline void convert(const CvMatMessage::ConstPtr &in,
                    PointCloudMessage::Ptr &out)
{
    const cv::Mat &matrix = in->value;

    if(in->getEncoding().matches(enc::pointXYZ)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for(int i = 0 ; i < matrix.rows ; ++i) {
            for(int j = 0 ; j < matrix.cols ; ++j) {
                pcl::PointXYZ pt;
                pt.x = matrix.at<float>(i, (j * 3 + 0));
                pt.y = matrix.at<float>(i, (j * 3 + 1));
                pt.z = matrix.at<float>(i, (j * 3 + 2));
                cloud->push_back(pt);
            }
        }

        cloud->height = matrix.rows;
        cloud->width  = matrix.cols;
        cloud->is_dense = true;
        cloud->header.frame_id = in->frame_id;
        cloud->header.stamp = in->stamp_micro_seconds;

        out->value = cloud;
    } else if(in->getEncoding().matches(enc::pointXYZI)) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for(int i = 0 ; i < matrix.rows ; ++i) {
            for(int j = 0 ; j < matrix.cols ; ++j) {
                pcl::PointXYZI pt;
                pt.x = matrix.at<float>(i, (j * 4 + 0));
                pt.y = matrix.at<float>(i, (j * 4 + 1));
                pt.z = matrix.at<float>(i, (j * 4 + 2));
                pt.intensity = matrix.at<float>(i, (j * 4 + 3));
                cloud->push_back(pt);
            }
        }

        cloud->height = matrix.rows;
        cloud->width  = matrix.cols;
        cloud->is_dense = true;
        cloud->header.frame_id = in->frame_id;
        cloud->header.stamp = in->stamp_micro_seconds;

        out->value = cloud;
    } else if(in->getEncoding().matches(enc::pointXYZRGB)) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


        for(int i = 0 ; i < matrix.rows ; ++i) {
            for(int j = 0 ; j < matrix.cols ; ++j) {
                pcl::PointXYZRGB pt;
                pt.x = matrix.at<float>(i, (j * 6 + 0));
                pt.y = matrix.at<float>(i, (j * 6 + 1));
                pt.z = matrix.at<float>(i, (j * 6 + 2));
                pt.r = matrix.at<float>(i, (j * 6 + 3));
                pt.g = matrix.at<float>(i, (j * 6 + 4));
                pt.b = matrix.at<float>(i, (j * 6 + 5));
                cloud->push_back(pt);
            }
        }

        cloud->height = matrix.rows;
        cloud->width  = matrix.cols;
        cloud->is_dense = true;
        cloud->header.frame_id = in->frame_id;
        cloud->header.stamp = in->stamp_micro_seconds;

        out->value = cloud;
    } else {
        throw std::runtime_error("Unsupported encoding!");
    }
    std::cout << std::endl;
}
}

PointmatrixToPointcloud::PointmatrixToPointcloud()
{
}

void PointmatrixToPointcloud::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    PointCloudMessage::Ptr out(new PointCloudMessage(in->frame_id, in->stamp_micro_seconds));
    implementation::convert(in, out);
    msg::publish(output_, out);
}

void PointmatrixToPointcloud::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("Point Matrix");
    output_ = node_modifier.addOutput<PointCloudMessage>("PointCloud");
}

void PointmatrixToPointcloud::setupParameters(Parameterizable& parameters)
{
}
