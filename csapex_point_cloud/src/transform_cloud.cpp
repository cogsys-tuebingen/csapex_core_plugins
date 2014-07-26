/// HEADER
#include "transform_cloud.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/transform_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/mpl/for_each.hpp>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <pcl_ros/transforms.h>

CSAPEX_REGISTER_CLASS(csapex::TransformCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformCloud::TransformCloud()
{
}

void TransformCloud::setup()
{
    input_cloud_ = modifier_->addInput<PointCloudMessage>("PointCloud");
    input_transform_ = modifier_->addInput<TransformMessage>("Transformation");

    output_ = modifier_->addOutput<PointCloudMessage>("PointCloud");
}

void TransformCloud::process()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<TransformCloud>(this, msg), msg->value);
}

template <class PointT>
void TransformCloud::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    TransformMessage::Ptr transform = input_transform_->getMessage<TransformMessage>();
    const tf::Transform& t = transform->value;

    typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
    pcl_ros::transformPointCloud(*cloud, *out, t);

    PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id));
    msg->value = out;

    output_->publish(msg);
}
