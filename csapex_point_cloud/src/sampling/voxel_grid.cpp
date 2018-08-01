/// HEADER
#include "voxel_grid.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <boost/mpl/for_each.hpp>
#include <csapex/utility/suppress_warnings_start.h>
    #include <pcl_ros/transforms.h>
    #include <pcl/filters/voxel_grid.h>
#include <csapex/utility/suppress_warnings_end.h>

CSAPEX_REGISTER_CLASS(csapex::VoxelGrid, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

VoxelGrid::VoxelGrid()
{
}

void VoxelGrid::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("resolution", 0.01, 1.0, 0.1, 0.005));
    parameters.addParameter(param::factory::declareBool("remove NAN", false), remove_nan_);
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

    typename pcl::PointCloud<PointT>::ConstPtr in;

    if(remove_nan_) {
        typename pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
        tmp->points.reserve(cloud->points.size());

        for(auto it = cloud->begin(); it != cloud->end(); ++it) {
            if(!std::isnan(it->x)) {
                tmp->points.push_back(*it);
            }
        }
        in = tmp;

    } else {
        in = cloud;
    }

    pcl::VoxelGrid<PointT> voxel_f;
    voxel_f.setInputCloud(in);
    voxel_f.setLeafSize(leaf);
    voxel_f.filter(*out);

    PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
    out->header = cloud->header;
    msg->value = out;

    msg::publish(output_, msg);
}
