#include "index_filter.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_point_cloud/indeces_message.h>
#include <utils_param/parameter_factory.h>
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

void IndexFilter::setup()
{
    input_cloud_ = modifier_->addInput<PointCloudMessage>("PointCloud");
    indeces_input_ = modifier_->addInput<PointIndecesMessage>("Indeces");
    output_cloud_ = modifier_->addOutput<PointCloudMessage>("Pointcloud");
}

void IndexFilter::process()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());
    boost::apply_visitor (PointCloudMessage::Dispatch<IndexFilter>(this, msg), msg->value);
}

template <class PointT>
void IndexFilter::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    PointIndecesMessage::Ptr indeces(indeces_input_->getMessage<PointIndecesMessage>());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>(*cloud, indeces->value->indices));

    PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
    out->value = cloud_filtered;
    output_cloud_->publish(out);
}
