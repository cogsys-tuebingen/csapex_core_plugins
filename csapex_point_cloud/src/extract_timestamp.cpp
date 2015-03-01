/// HEADER
#include "extract_timestamp.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::ExtractTimeStampCloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ExtractTimeStampCloud::ExtractTimeStampCloud()
{
}

void ExtractTimeStampCloud::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");

    output_ = node_modifier.addOutput<TimeStampMessage>("Time");
    output_frame_ = node_modifier.addOutput<std::string>("Target Frame");
}

void ExtractTimeStampCloud::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

    boost::apply_visitor (PointCloudMessage::Dispatch<ExtractTimeStampCloud>(this, msg), msg->value);
}

template <class PointT>
void ExtractTimeStampCloud::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    time->value = time->value.fromNSec(cloud->header.stamp * 1000);
    msg::publish(output_, time);

    msg::publish(output_frame_, cloud->header.frame_id);
}
