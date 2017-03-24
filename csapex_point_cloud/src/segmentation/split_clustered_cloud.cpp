#include "split_clustered_cloud.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/msg/indices_message.h>

/// PCL
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif //__clang__
#include <pcl/filters/extract_indices.h>
#if __clang__
#pragma clang diagnostic pop
#endif //__clang__

CSAPEX_REGISTER_CLASS(csapex::SplitClusteredCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SplitClusteredCloud::SplitClusteredCloud()
{
}

void SplitClusteredCloud::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

    boost::apply_visitor (PointCloudMessage::Dispatch<SplitClusteredCloud>(this, msg), msg->value);
}

void SplitClusteredCloud::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_ = node_modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Clusters");

    output1_ = node_modifier.addOutput<PointCloudMessage>("PointCloud1");
    output2_ = node_modifier.addOutput<PointCloudMessage>("PointCloud2");
    output3_ = node_modifier.addOutput<PointCloudMessage>("PointCloud3");
    output4_ = node_modifier.addOutput<PointCloudMessage>("PointCloud4");
}

template <class PointT>
void SplitClusteredCloud::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{

    std::vector<PointCloudMessage::Ptr> out_msgs;
    std::shared_ptr<std::vector<pcl::PointIndices> const> cluster_indices;
    cluster_indices = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_indices_);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices->begin(); it != cluster_indices->end (); ++it)
    {
        // for every cluster
        // extract the points of the cluster
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header = cloud->header;

        PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        out->value = cloud_cluster;
        out_msgs.push_back(out);
    }


    if (out_msgs.size() >= 1) msg::publish(output1_, out_msgs.at(0));
    if (out_msgs.size() >= 2) msg::publish(output2_, out_msgs.at(1));
    if (out_msgs.size() >= 3) msg::publish(output3_, out_msgs.at(2));
    if (out_msgs.size() >= 4) msg::publish(output4_, out_msgs.at(3));
}
