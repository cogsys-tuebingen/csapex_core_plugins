/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>

/// PROJECT
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_laser_processing/data/segment.h>

/// SYSTEM
#include <sensor_msgs/LaserScan.h>

CSAPEX_REGISTER_CLASS(csapex::RegisterScan2DPlugin, csapex::CorePlugin)

using namespace csapex;

Q_DECLARE_METATYPE(lib_laser_processing::Scan)
Q_DECLARE_METATYPE(lib_laser_processing::LabeledScan)


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

    static connection_types::ScanMessage::Ptr ros2apex(const sensor_msgs::LaserScan::ConstPtr &ros_msg) {
        connection_types::ScanMessage::Ptr out(new connection_types::ScanMessage);
        out->value = lib_laser_processing::Scan(ros_msg->ranges, ros_msg->angle_min, ros_msg->angle_increment);
        copy(*ros_msg, out->value);
        out->value.header.stamp_nsec = ros_msg->header.stamp.toNSec();
        out->stamp_micro_seconds = ros_msg->header.stamp.toNSec();
        return out;
    }
    static sensor_msgs::LaserScan::Ptr apex2ros(const connection_types::ScanMessage::ConstPtr& apex_msg) {
        sensor_msgs::LaserScan::Ptr out(new sensor_msgs::LaserScan);
        copy(apex_msg->value, *out);
        apex_msg->value.getRanges(out->ranges);
        out->header.stamp = out->header.stamp.fromNSec(apex_msg->value.header.stamp_nsec);
        return out;
    }
};

RegisterScan2DPlugin::RegisterScan2DPlugin()
{
}

void RegisterScan2DPlugin::init(CsApexCore& core)
{
    qRegisterMetaType < lib_laser_processing::Scan::Ptr > ("lib_laser_processing::Scan::Ptr");
    qRegisterMetaType < lib_laser_processing::LabeledScan::Ptr > ("lib_laser_processing::LabeledScan::Ptr");

    RosMessageConversion::registerConversion<sensor_msgs::LaserScan, connection_types::ScanMessage, ConvertScan>();
}

void RegisterScan2DPlugin::shutdown()
{

}
