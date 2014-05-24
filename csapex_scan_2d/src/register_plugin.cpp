/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>

/// PROJECT
#include <csapex/manager/connection_type_manager.h>
#include <csapex/model/tag.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <sensor_msgs/LaserScan.h>

CSAPEX_REGISTER_CLASS(csapex::RegisterScan2DPlugin, csapex::CorePlugin)

using namespace csapex;


struct ConvertScan
{
    template <typename A, typename B>
    static void copy(A& a, B& b) {
        b.header.frame_id = a.header.frame_id;
        b.header.seq      = a.header.seq;
        b.angle_min       = a.angle_min;
        b.angle_max       = a.angle_max;
        b.angle_increment = a.angle_increment;
        b.range_min       = a.range_min;
        b.range_max       = a.range_max;
    }

    static void ros2apex(const sensor_msgs::LaserScan::ConstPtr &ros_msg, connection_types::ScanMessage::Ptr& out) {
        copy(*ros_msg, out->value);
        out->value = lib_laser_processing::Scan(ros_msg->ranges, ros_msg->angle_min, ros_msg->angle_increment);
        out->value.header.stamp_nsec = ros_msg->header.stamp.toNSec();
    }
    static void apex2ros(const connection_types::ScanMessage::Ptr& apex_msg, sensor_msgs::LaserScan::Ptr &out) {
        copy(apex_msg->value, *out);
        apex_msg->value.getRanges(out->ranges);
        out->header.stamp = out->header.stamp.fromNSec(apex_msg->value.header.stamp_nsec);
    }
};

RegisterScan2DPlugin::RegisterScan2DPlugin()
{
}

void RegisterScan2DPlugin::init(CsApexCore& core)
{
    Tag::createIfNotExists("Features");

    ConnectionTypeManager::registerMessage<connection_types::ScanMessage>();
    ConnectionTypeManager::registerMessage<connection_types::LabeledScanMessage>();

    RosMessageConversion::registerConversion<sensor_msgs::LaserScan, connection_types::ScanMessage, ConvertScan>();
}
