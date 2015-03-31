/// HEADER
#include "transform_cloud.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_transform/transform_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::TransformCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformCloud::TransformCloud()
{
}

void TransformCloud::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    input_transform_ = node_modifier.addInput<TransformMessage>("Transformation");

    output_ = node_modifier.addOutput<PointCloudMessage>("PointCloud");
}

void TransformCloud::process()
{
    PointCloudMessage::ConstPtr msg = msg::getMessage<PointCloudMessage>(input_cloud_);

    boost::apply_visitor (PointCloudMessage::Dispatch<TransformCloud>(this, msg), msg->value);
}

template <class PointT>
void TransformCloud::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    TransformMessage::ConstPtr transform = msg::getMessage<TransformMessage>(input_transform_);
    const tf::Transform& t = transform->value;

    typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
    out->header = cloud->header;
    out->width = cloud->width;
    out->height = cloud->height;
    out->is_dense = cloud->is_dense;


    std::size_t N = cloud->points.size();
    out->points.resize(N);

    for(std::size_t i = 0; i < N; ++i) {
        PointT pt = cloud->points[i];
        tf::Quaternion q = t.getRotation();
        tf::Vector3 tr = t.getOrigin();

        tf::Vector3 p(pt.x, pt.y, pt.z);

//        tf::Vector3 tfd =  tf::quatRotate(q, p) + tr;
        tf::Vector3 tfd = t * p;
        pt.x = tfd.x();
        pt.y = tfd.y();
        pt.z = tfd.z();
        out->points[i] = pt;
    }

    std::string frame = cloud->header.frame_id;

    if(frame == transform->child_frame) {
        frame = transform->frame_id;
    }

    PointCloudMessage::Ptr msg(new PointCloudMessage(frame, cloud->header.stamp));
    msg->value = out;

    msg::publish(output_, msg);
}
