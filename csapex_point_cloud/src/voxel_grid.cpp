/// HEADER
#include "voxel_grid.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/mpl/for_each.hpp>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

CSAPEX_REGISTER_CLASS(csapex::VoxelGrid, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

VoxelGrid::VoxelGrid()
{
    addTag(Tag::get("PointCloud"));

    addParameter(param::ParameterFactory::declareRange("resolution", 0.01, 1.0, 0.1, 0.01));
}

void VoxelGrid::setup()
{
    input_cloud_ = addInput<PointCloudMessage>("PointCloud");

    output_ = addOutput<PointCloudMessage>("PointCloud");
}

void VoxelGrid::process()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<VoxelGrid>(this), msg->value);
}

template <class PointT>
void VoxelGrid::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    double res = param<double>("resolution");
    Eigen::Vector4f leaf(res, res, res, 0);

    typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> voxel_f;
    voxel_f.setInputCloud(cloud);
    voxel_f.setLeafSize(leaf);
    voxel_f.filter(*out);

    PointCloudMessage::Ptr msg(new PointCloudMessage);
    msg->value = out;

    output_->publish(msg);
}
