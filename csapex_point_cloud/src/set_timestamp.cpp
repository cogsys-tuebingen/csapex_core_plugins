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
    input_frame_ = modifier_->addOptionalInput<connection_types::GenericValueMessage<std::string> >("Frame");
    input_time_ = modifier_->addInput<connection_types::TimeStampMessage>("Time");

    output_ = modifier_->addOutput<connection_types::PointCloudMessage>("PointCloud");
}

void SetTimeStamp::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<SetTimeStamp>(this, msg), msg->value);
}

template <class PointT>
void SetTimeStamp::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    connection_types::TimeStampMessage::Ptr time = input_time_->getMessage<connection_types::TimeStampMessage>();
    cloud->header.stamp = time->value.toNSec() / 1e3; // is this 1e6?

    connection_types::PointCloudMessage::Ptr msg(new connection_types::PointCloudMessage(cloud->header.frame_id, time->value.toNSec()));

    if(input_frame_->hasMessage()) {
        cloud->header.frame_id = input_frame_->getMessage<connection_types::GenericValueMessage<std::string> >()->value;
    }

    msg->value = cloud;
    output_->publish(msg);
}
