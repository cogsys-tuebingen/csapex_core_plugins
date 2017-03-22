/// HEADER
#include "passthrough_by_cluster.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/indeces_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <pcl/filters/passthrough.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

CSAPEX_REGISTER_CLASS(csapex::PassThroughByCluster, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PassThroughByCluster::PassThroughByCluster()
{
}

void PassThroughByCluster::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("keep organized", true));
}

void PassThroughByCluster::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    input_clusters_ = node_modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Clusters");

    output_pos_ = node_modifier.addOutput<PointCloudMessage>("filtered PointCloud (+)");
    output_neg_ = node_modifier.addOutput<PointCloudMessage>("filtered PointCloud (-)");
}

void PassThroughByCluster::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));

    boost::apply_visitor (PointCloudMessage::Dispatch<PassThroughByCluster>(this, msg), msg->value);
}

template <class PointT>
void PassThroughByCluster::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::shared_ptr<std::vector<pcl::PointIndices> const> input_clusters =
            msg::getMessage<GenericVectorMessage, pcl::PointIndices>(input_clusters_);

    // merge all cluster indices
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    for(const pcl::PointIndices &i : *input_clusters) {
        indices->indices.insert(indices->indices.end(), i.indices.begin(), i.indices.end());
    }

    // check available fields!
    pcl::ExtractIndices<PointT> pass;
    pass.setIndices(indices);

    pass.setInputCloud(cloud);
    pass.setKeepOrganized(readParameter<bool>("keep organized"));

    if(msg::isConnected(output_pos_)) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_pos_, msg);
    }

    if(msg::isConnected(output_neg_)) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.setNegative(true);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_neg_, msg);
    }

}
