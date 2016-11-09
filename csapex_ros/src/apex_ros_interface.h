#ifndef APEX_ROS_INTERFACE_H
#define APEX_ROS_INTERFACE_H

/// PROJECT
#include <csapex/core/core_plugin.h>
#include <csapex/signal/signal_fwd.h>

/// SYSTEM
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>

// TODO:
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
    void setupGraph(SubgraphNode *graph);
    void shutdown() override;

private:
    void registerCommandListener();
    void registerClockWatchdog();

    void clock(const rosgraph_msgs::ClockConstPtr& clock);
    void command(const std_msgs::StringConstPtr &cmd, bool global_cmd);
    void loadParameterValue(const std::string &prefix, const std::string &parameter_name, const XmlRpc::XmlRpcValue &parameter_value);

private:
    ros::Subscriber global_command_sub_;
    ros::Subscriber private_command_sub_;

    ros::Subscriber clock_sub_;

    CsApexCore* core_;

    bool disabled_;

    ros::Time last_clock_;
    EventPtr clock_reset_event_;

    slim_signal::ScopedConnection connection_;
};
}
#endif // APEX_ROS_INTERFACE_H
