#ifndef APEX_ROS_INTERFACE_H
#define APEX_ROS_INTERFACE_H

/// PROJECT
#include <csapex/core/core_plugin.h>

/// SYSTEM
#include <ros/ros.h>
#include <std_msgs/String.h>

// TODO:
// * make this an apex core plugin
//  * advertise service to pause/enable apex
// * move ros specific stuff here

namespace csapex
{
class APEXRosInterface : public CorePlugin
{
public:
    APEXRosInterface();
    ~APEXRosInterface();
    void prepare(Settings& settings);
    void init(CsApexCore& core);
    void shutdown();

private:
    void registerCommandListener();
    void command(const std_msgs::StringConstPtr &cmd, bool global_cmd);

private:
    ros::Subscriber global_command_sub_;
    ros::Subscriber private_command_sub_;
    CsApexCore* core_;

    bool disabled_;
};
}
#endif // APEX_ROS_INTERFACE_H
