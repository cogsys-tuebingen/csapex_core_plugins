#include "indexed_pointcloud.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::IndexedPointCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

IndexedPointCloud::IndexedPointCloud()
{
    addTag(Tag::get("PointCloud"));
}

void IndexedPointCloud::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("Depth Image");
    output_ = modifier_->addOutput<PointCloudMessage>("Indexed Pointcloud");
}

void IndexedPointCloud::process()
{
    CvMatMessage::Ptr input(input_->getMessage<CvMatMessage>());
    if(input->value.type() != CV_32FC1) {
        throw std::runtime_error("Unsupported depth image type!");
    }

    int cols = input->value.cols;
    int rows = input->value.rows;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
            (new pcl::PointCloud<pcl::PointXYZ>(cols, rows));

    const float   *src_ptr = input->value.ptr<float>();
    pcl::PointXYZ *dst_ptr = cloud->points.data();
    for(int y = 0 ; y < rows ; ++y) {
        for(int x = 0 ; x < cols ; ++x) {
            int pos = y * cols + x;
            dst_ptr[pos].x = x;
            dst_ptr[pos].y = y;
            dst_ptr[pos].z = src_ptr[pos] != src_ptr[pos] ?
                             0.f : src_ptr[pos];
        }
    }

    PointCloudMessage::Ptr output(new PointCloudMessage(cloud->header.frame_id));
    output->value = cloud;
    output_->publish(output);
}
