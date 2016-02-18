/// HEADER
#include "set_timestamp.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::SetTimeStamp, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SetTimeStamp::SetTimeStamp()
{
}

void SetTimeStamp::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<connection_types::PointCloudMessage>("PointCloud");
    input_frame_ = node_modifier.addOptionalInput<std::string>("Frame");
    input_time_ = node_modifier.addInput<connection_types::RosTimeStampMessage>("Time");

    output_ = node_modifier.addOutput<connection_types::PointCloudMessage>("PointCloud");
}

void SetTimeStamp::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

    boost::apply_visitor (PointCloudMessage::Dispatch<SetTimeStamp>(this, msg), msg->value);
}

template <class PointT>
void SetTimeStamp::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    connection_types::RosTimeStampMessage::ConstPtr time = msg::getMessage<connection_types::RosTimeStampMessage>(input_time_);

    connection_types::PointCloudMessage::Ptr msg(new connection_types::PointCloudMessage(cloud->header.frame_id, time->value.toNSec()));

    typename pcl::PointCloud<PointT>::Ptr cloud_copy(new pcl::PointCloud<PointT>);
    *cloud_copy = *cloud;
    msg->value = cloud_copy;

    typename pcl::PointCloud<PointT>::Ptr out_cloud = boost::get<typename pcl::PointCloud<PointT>::Ptr>(msg->value);

    if(msg::hasMessage(input_frame_)) {
        out_cloud->header.frame_id = msg::getValue<std::string>(input_frame_);
    }
    out_cloud->header.stamp = time->value.toNSec() / 1e3; // microseconds

    msg::publish(output_, msg);
}
