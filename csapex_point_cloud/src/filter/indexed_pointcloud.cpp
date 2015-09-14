#include "indexed_pointcloud.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::IndexedPointCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

IndexedPointCloud::IndexedPointCloud()
{
}

void IndexedPointCloud::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("Depth Image");
    output_ = node_modifier.addOutput<PointCloudMessage>("Indexed Pointcloud");
}

void IndexedPointCloud::process()
{
    CvMatMessage::ConstPtr input(msg::getMessage<CvMatMessage>(input_));
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

    PointCloudMessage::Ptr output(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
    output->value = cloud;
    msg::publish(output_, output);
}
