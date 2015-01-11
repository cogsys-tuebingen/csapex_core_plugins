/// HEADER
#include "set_timestamp.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::SetTimeStamp, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SetTimeStamp::SetTimeStamp()
{
}

void SetTimeStamp::setup()
{
    input_ = modifier_->addInput<connection_types::PointCloudMessage>("PointCloud");
    input_frame_ = modifier_->addOptionalInput<std::string>("Frame");
    input_time_ = modifier_->addInput<connection_types::TimeStampMessage>("Time");

    output_ = modifier_->addOutput<connection_types::PointCloudMessage>("PointCloud");
}

void SetTimeStamp::process()
{
    PointCloudMessage::ConstPtr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<SetTimeStamp>(this, msg), msg->value);
}

template <class PointT>
void SetTimeStamp::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    connection_types::TimeStampMessage::ConstPtr time = input_time_->getMessage<connection_types::TimeStampMessage>();

    connection_types::PointCloudMessage::Ptr msg(new connection_types::PointCloudMessage(cloud->header.frame_id, time->value.toNSec()));

    typename pcl::PointCloud<PointT>::Ptr cloud_copy(new pcl::PointCloud<PointT>);
    *cloud_copy = *cloud;
    msg->value = cloud_copy;

    typename pcl::PointCloud<PointT>::Ptr out_cloud = boost::get<typename pcl::PointCloud<PointT>::Ptr>(msg->value);

    if(input_frame_->hasMessage()) {
        out_cloud->header.frame_id = input_frame_->getValue<std::string>();
    }
    out_cloud->header.stamp = time->value.toNSec() / 1e3; // microseconds

    output_->publish(msg);
}
