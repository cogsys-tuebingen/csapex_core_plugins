/// HEADER
#include "voxel_grid.h"

/// PROJECT
#include <csapex/msg/io.h>
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
}

void VoxelGrid::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("resolution", 0.01, 1.0, 0.1, 0.01));
}

void VoxelGrid::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");

    output_ = node_modifier.addOutput<PointCloudMessage>("PointCloud");
}

void VoxelGrid::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));

    boost::apply_visitor (PointCloudMessage::Dispatch<VoxelGrid>(this, msg), msg->value);
}

template <class PointT>
void VoxelGrid::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    double res = readParameter<double>("resolution");
    Eigen::Vector4f leaf(res, res, res, 0);

    typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> voxel_f;
    voxel_f.setInputCloud(cloud);
    voxel_f.setLeafSize(leaf);
    voxel_f.filter(*out);

    PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
    msg->value = out;

    msg::publish(output_, msg);
}
