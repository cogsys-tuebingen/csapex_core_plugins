#include "index_filter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <pcl/point_types.h>

CSAPEX_REGISTER_CLASS(csapex::IndexFilter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

IndexFilter::IndexFilter()
{
}

void IndexFilter::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    indices_input_ = node_modifier.addInput<PointIndicesMessage>("indices");
    output_cloud_ = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
}

void IndexFilter::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<IndexFilter>(this, msg), msg->value);
}

template <class PointT>
void IndexFilter::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    PointIndicesMessage::ConstPtr indices(msg::getMessage<PointIndicesMessage>(indices_input_));
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>(*cloud, indices->value->indices));

    PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
    out->value = cloud_filtered;
    msg::publish(output_cloud_, out);
}
