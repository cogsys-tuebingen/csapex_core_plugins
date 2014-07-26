/// HEADER
#include "voxel_grid.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <boost/mpl/for_each.hpp>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

CSAPEX_REGISTER_CLASS(csapex::VoxelGrid, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

VoxelGrid::VoxelGrid()
{
    addParameter(param::ParameterFactory::declareRange("resolution", 0.01, 1.0, 0.1, 0.01));
}

void VoxelGrid::setup()
{
    input_cloud_ = modifier_->addInput<PointCloudMessage>("PointCloud");

    output_ = modifier_->addOutput<PointCloudMessage>("PointCloud");
}

void VoxelGrid::process()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<VoxelGrid>(this, msg), msg->value);
}

template <class PointT>
void VoxelGrid::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    double res = readParameter<double>("resolution");
    Eigen::Vector4f leaf(res, res, res, 0);

    typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> voxel_f;
    voxel_f.setInputCloud(cloud);
    voxel_f.setLeafSize(leaf);
    voxel_f.filter(*out);

    PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id));
    msg->value = out;

    output_->publish(msg);
}
