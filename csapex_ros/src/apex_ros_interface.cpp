/// HEADER
#include "apex_ros_interface.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ros/ros_handler.h>
#include "import_ros.h"
#include <csapex/model/graph.h>
#include <csapex/model/node_state.h>
#include <csapex/factory/message_factory.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <boost/regex.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <QMimeData>

CSAPEX_REGISTER_CLASS(csapex::APEXRosInterface, csapex::CorePlugin)

using namespace csapex;

template <typename RosType, typename ApexType>
struct ConvertIntegral
{
    static typename connection_types::GenericValueMessage<ApexType>::Ptr ros2apex(const typename RosType::ConstPtr &ros_msg) {
        typename connection_types::GenericValueMessage<ApexType>::Ptr out(new connection_types::GenericValueMessage<ApexType>);
        out->value = ros_msg->data;
        return out;
    }
    static typename RosType::Ptr apex2ros(const typename connection_types::GenericValueMessage<ApexType>::ConstPtr& apex_msg) {
        typename RosType::Ptr out(new RosType);
        out->data = apex_msg->value;
        return out;
    }
};




APEXRosInterface::APEXRosInterface()
    : core_(nullptr), disabled_(false)
{
}

APEXRosInterface::~APEXRosInterface()
{
}

void APEXRosInterface::prepare(Settings &settings)
{
    ROSHandler::createInstance(settings);
}

void APEXRosInterface::init(CsApexCore &core)
{
    core_ = &core;

    ROSHandler::instance().registerConnectionCallback(std::bind(&APEXRosInterface::registerCommandListener, this));

    RosMessageConversion::registerConversion<std_msgs::Bool, connection_types::GenericValueMessage<bool>, ConvertIntegral<std_msgs::Bool, bool> >();
    RosMessageConversion::registerConversion<std_msgs::Int32, connection_types::GenericValueMessage<int>, ConvertIntegral<std_msgs::Int32, int> >();
    RosMessageConversion::registerConversion<std_msgs::Float64, connection_types::GenericValueMessage<double>, ConvertIntegral<std_msgs::Float64, double> >();
    RosMessageConversion::registerConversion<std_msgs::String, connection_types::GenericValueMessage<std::string>, ConvertIntegral<std_msgs::String, std::string> >();
}

void APEXRosInterface::registerCommandListener()
{
    assert(ROSHandler::instance().isConnected());
    global_command_sub_ = ROSHandler::instance().nh()->subscribe
            <std_msgs::String>("/syscommand", 10, std::bind(&APEXRosInterface::command, this, std::placeholders::_1, true));
    private_command_sub_ = ROSHandler::instance().nh()->subscribe
            <std_msgs::String>("command", 10, std::bind(&APEXRosInterface::command, this, std::placeholders::_1, false));

    ros::spinOnce();
}

void APEXRosInterface::command(const std_msgs::StringConstPtr& cmd, bool global_cmd)
{
    std::string command = cmd->data;
    bool local_cmd = !global_cmd;

    /*
     * pause / unpause:
     *   - temporary disable everything to conserve computational power
     *   - globally accepted, if this node is not disabled
     *
     * stop / resume
     *   - completely disable / enable this subsystem, when it is not needed
     *   - only locally accepted
     *   - overwrites pause -> a disabled instance cannot be unpaused
     */

    if(!disabled_) {
        // disabled state is stronger than pause / unpause
        if(command == "pause") {
            core_->setPause(true);

        } else if(command == "unpause"){
            core_->setPause(false);
        }
    }

    if(local_cmd) {
        if(command == "stop") {
            disabled_ = true;
            core_->setPause(true);
        } else if(command == "resume") {
            disabled_ = false;
            core_->setPause(false);
        }
    }
}

void APEXRosInterface::shutdown()
{
    ROSHandler::instance().stop();
}
