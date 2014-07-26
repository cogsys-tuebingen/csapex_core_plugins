/// HEADER
#include "extract_timestamp.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ExtractTimeStamp, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ExtractTimeStamp::ExtractTimeStamp()
{
}

void ExtractTimeStamp::setup()
{
    input_ = modifier_->addInput<PointCloudMessage>("PointCloud");

    output_ = modifier_->addOutput<TimeStampMessage>("Time");
    output_frame_ = modifier_->addOutput<GenericValueMessage<std::string> >("Target Frame");
}

void ExtractTimeStamp::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<ExtractTimeStamp>(this, msg), msg->value);
}

template <class PointT>
void ExtractTimeStamp::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    time->value = time->value.fromNSec(cloud->header.stamp * 1000);
    output_->publish(time);

    connection_types::GenericValueMessage<std::string>::Ptr frame(new connection_types::GenericValueMessage<std::string>);
    frame->value = cloud->header.frame_id;
    output_frame_->publish(frame);
}
