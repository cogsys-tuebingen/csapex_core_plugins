/// HEADER
#include "merge_clouds.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::MergeClouds, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MergeClouds::MergeClouds()
{
}

void MergeClouds::setupParameters(Parameterizable& parameters)
{
}

void MergeClouds::setup(NodeModifier& node_modifier)
{
    in_a_ = node_modifier.addInput<PointCloudMessage>("Cloud A");
    in_b_ = node_modifier.addInput<PointCloudMessage>("Cloud B");

    out_ = node_modifier.addOutput<PointCloudMessage>("merged PointCloud");
}

void MergeClouds::process()
{
    result_.reset(new pcl::PointCloud<pcl::PointXYZL>);

    PointCloudMessage::ConstPtr msg_a(msg::getMessage<PointCloudMessage>(in_a_));
    boost::apply_visitor (PointCloudMessage::Dispatch<MergeClouds>(this, msg_a), msg_a->value);

    PointCloudMessage::ConstPtr msg_b(msg::getMessage<PointCloudMessage>(in_b_));
    boost::apply_visitor (PointCloudMessage::Dispatch<MergeClouds>(this, msg_a), msg_b->value);

    PointCloudMessage::Ptr output(new PointCloudMessage(msg_a->frame_id, msg_a->stamp_micro_seconds));
    output->value = result_;
    msg::publish(out_, output);
}

namespace
{
template <typename B>
void set(pcl::PointXYZL& a, const B& b)
{
    a.x = b.x;
    a.y = b.y;
    a.z = b.z;
    a.label = 0;
}
template <>
void set<pcl::PointXYZL>(pcl::PointXYZL& a, const pcl::PointXYZL& b)
{
    a = b;
}
}

template <class PointT>
void MergeClouds::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    result_->points.reserve(result_->points.size() + cloud->points.size());
    for(const auto& pt : cloud->points) {
        pcl::PointXYZL p;
        set(p, pt);
        result_->points.push_back(p);
    }
}
