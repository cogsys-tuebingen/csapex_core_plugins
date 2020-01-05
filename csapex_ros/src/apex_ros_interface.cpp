/// HEADER
#include "apex_ros_interface.h"

/// PROJECT
#include "import_ros.h"
#include <csapex/factory/message_factory.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/token.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/no_message.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex_ros/ros_handler.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <QMimeData>
#include <boost/algorithm/string/replace.hpp>
#include <chrono>
#include <console_bridge/console.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

CSAPEX_REGISTER_CLASS(csapex::APEXRosInterface, csapex::CorePlugin)

using namespace csapex;

template <typename RosType, typename ApexType>
struct ConvertIntegral
{
    static typename connection_types::GenericValueMessage<ApexType>::Ptr ros2apex(const typename RosType::ConstPtr& ros_msg)
    {
        typename connection_types::GenericValueMessage<ApexType>::Ptr out(new connection_types::GenericValueMessage<ApexType>);
        out->value = ros_msg->data;
        return out;
    }
    static typename RosType::Ptr apex2ros(const typename connection_types::GenericValueMessage<ApexType>::ConstPtr& apex_msg)
    {
        typename RosType::Ptr out(new RosType);
        out->data = apex_msg->value;
        return out;
    }
};

struct ConvertTimeStamp
{
    static typename connection_types::TimestampMessage::Ptr ros2apex(const typename rosgraph_msgs::Clock::ConstPtr& ros_msg)
    {
        std::chrono::nanoseconds ns(ros_msg->clock.toNSec());
        auto ms = std::chrono::duration_cast<std::chrono::microseconds>(ns);
        connection_types::TimestampMessage::Tp tp = connection_types::TimestampMessage::Tp(ms);

        return std::make_shared<connection_types::TimestampMessage>(tp);
    }
    static typename rosgraph_msgs::Clock::Ptr apex2ros(const typename connection_types::TimestampMessage::ConstPtr& apex_msg)
    {
        typename rosgraph_msgs::Clock::Ptr out(new rosgraph_msgs::Clock);
        auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(apex_msg->value.time_since_epoch());
        out->clock.fromNSec(nsec.count());
        return out;
    }
};

APEXRosInterface::APEXRosInterface() : core_(nullptr), disabled_(false)
{
    last_clock_ = ros::Time(0);
}

APEXRosInterface::~APEXRosInterface()
{
}

void APEXRosInterface::prepare(Settings& settings)
{
    ROSHandler::createInstance(settings);
}

void APEXRosInterface::init(CsApexCore& core)
{
    core_ = &core;

    if (core_->getSettings().get<bool>("debug", false)) {
        console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
    }

    core_->getSettings().setting_changed.connect([this](const std::string& name) {
        if (name == "debug") {
            if (core_->getSettings().get<bool>("debug")) {
                console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
            } else {
                console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_INFO);
            }
        }
    });

    auto init = [this]() {
        registerCommandListener();
        registerClockWatchdog();
    };
    if (ROSHandler::instance().isConnected()) {
        init();
    } else {
        connection_ = ROSHandler::instance().connected.connect(init);
    }

    connection_types::MessageConversionHook<connection_types::GenericPointerMessage, geometry_msgs::PoseArray>::registerConversion();
    connection_types::MessageConversionHook<connection_types::GenericPointerMessage, visualization_msgs::MarkerArray>::registerConversion();
    connection_types::MessageConversionHook<connection_types::GenericPointerMessage, visualization_msgs::Marker>::registerConversion();

    RosMessageConversion::registerConversion<rosgraph_msgs::Clock, connection_types::TimestampMessage, ConvertTimeStamp>();

    RosMessageConversion::registerConversion<std_msgs::Bool, connection_types::GenericValueMessage<bool>, ConvertIntegral<std_msgs::Bool, bool>>();
    RosMessageConversion::registerConversion<std_msgs::Int32, connection_types::GenericValueMessage<int>, ConvertIntegral<std_msgs::Int32, int>>();
    RosMessageConversion::registerConversion<std_msgs::Float64, connection_types::GenericValueMessage<double>, ConvertIntegral<std_msgs::Float64, double>>();
    RosMessageConversion::registerConversion<std_msgs::String, connection_types::GenericValueMessage<std::string>, ConvertIntegral<std_msgs::String, std::string>>();

    core_->loaded.connect([this]() {
        if (ROSHandler::instance().isConnected()) {
            XmlRpc::XmlRpcValue params, result, payload;
            params[0] = ros::this_node::getName();

            std::string prefix = ros::this_node::getName();

            if (ros::master::execute("getParamNames", params, result, payload, true)) {
                if (result.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                    return;
                }

                std::string name = result[1];
                if (name != "Parameter names") {
                    return;
                }

                XmlRpc::XmlRpcValue data = result[2];
                for (std::size_t i = 0, total = data.size(); i < total; ++i) {
                    std::string parameter_name = data[i];

                    if (parameter_name.substr(0, prefix.size()) != prefix) {
                        continue;
                    }

                    try {
                        XmlRpc::XmlRpcValue parameter_value;
                        ros::param::getCached(parameter_name, parameter_value);

                        loadParameterValue(prefix, parameter_name, parameter_value);
                    } catch (std::exception e) {
                        // silence
                    }
                }
            }
        }
    });
}

void APEXRosInterface::setupGraph(SubgraphNode* graph)
{
    clock_reset_event_ = graph->createInternalEvent(makeEmpty<connection_types::AnyMessage>(), graph->getGraph()->makeUUID("event_ros_time_reset"), "ros time reset");
}

void APEXRosInterface::loadParameterValue(const std::string& prefix, const std::string& parameter_name, const XmlRpc::XmlRpcValue& parameter_value)
{
    std::string apex_name = parameter_name.substr(prefix.size());

    GraphImplementationPtr graph = core_->getRoot()->getLocalGraph();

    std::vector<std::string> levels;

    std::cerr << "analyzing parameter " << parameter_name << std::endl;

    std::size_t delim = 0;
    while (delim != std::string::npos) {
        delim = apex_name.find('/');
        std::string subname = apex_name.substr(0, delim);
        if (!subname.empty()) {
            levels.push_back(subname);
        }
        apex_name = apex_name.substr(delim + 1);
    }

    if (levels.empty()) {
        return;
    }

    NodeHandle* nh = nullptr;
    std::string param_name;
    for (std::size_t i = 0, n = levels.size(); i < n - 1; ++i) {
        const std::string subname = levels.at(i);
        std::cerr << "searching for " << subname << std::endl;
        nh = graph->findNodeHandleWithLabel(subname);
        if (!nh) {
            std::cerr << "no parameter for " << parameter_name << ", label " << subname << " non-existent" << std::endl;
            return;
        }

        if (NodePtr node = nh->getNode().lock()) {
            std::ostringstream param_name_builder;
            std::copy(std::next(levels.begin(), i + 1), levels.end(), std::ostream_iterator<std::string>(param_name_builder, "/"));
            param_name = param_name_builder.str();
            param_name = param_name.substr(0, param_name.size() - 1);

            std::cerr << "searching for param " << param_name << " in node " << node->getUUID() << std::endl;
            if (node->hasParameter(param_name))
                break;
            boost::algorithm::replace_all(param_name, " ", "_");
            std::cerr << "searching for param " << param_name << " (fallback)" << std::endl;
            if (node->hasParameter(param_name))
                break;
        }
        if (i < n - 2) {
            auto subgraph = std::dynamic_pointer_cast<SubgraphNode>(nh->getNode().lock());
            if (!subgraph) {
                std::cerr << "no parameter for " << parameter_name << ", child " << subname << " is not a graph, but " << nh->getType() << std::endl;
                return;
            } else {
                graph = subgraph->getLocalGraph();
            }
        }
    }
    std::cerr << "found parameter " << param_name << std::endl;
    if (!nh) {
        std::cerr << "cannot set parameter " << param_name << ", no node handle exists" << std::endl;
        return;
    }

    NodePtr node = nh->getNode().lock();
    if (!node) {
        std::cerr << "cannot set parameter " << param_name << ", no node exists" << std::endl;
        return;
    }

    if (!node->hasParameter(param_name)) {
        std::cerr << "node " << nh->getUUID() << " doesn't have a parameter called " << param_name << std::endl;
        return;
    }

    param::ParameterPtr p = node->getMappedParameter(param_name);

    switch (parameter_value.getType()) {
        case XmlRpc::XmlRpcValue::TypeInt: {
            int val;
            ros::param::get(parameter_name, val);
            std::cerr << "setting int parameter for " << nh->getUUID() << ":" << param_name << std::endl;
            p->set(val);
        } break;
        case XmlRpc::XmlRpcValue::TypeDouble: {
            double val;
            ros::param::get(parameter_name, val);
            std::cerr << "setting double parameter for " << nh->getUUID() << ":" << param_name << std::endl;
            p->set(val);
        } break;
        case XmlRpc::XmlRpcValue::TypeBoolean: {
            bool val;
            ros::param::get(parameter_name, val);
            std::cerr << "setting bool parameter for " << nh->getUUID() << ":" << param_name << std::endl;
            p->set(val);
        } break;
        case XmlRpc::XmlRpcValue::TypeString: {
            std::string val;
            ros::param::get(parameter_name, val);
            std::cerr << "setting string parameter for " << nh->getUUID() << ":" << param_name << std::endl;
            p->set(val);
        } break;

        default:
            std::cerr << "cannot set parameter for " << nh->getUUID() << ":" << param_name << " is of unknown type " << (int)parameter_value.getType() << std::endl;
            break;
    }
}

void APEXRosInterface::registerCommandListener()
{
    assert(ROSHandler::instance().isConnected());
    global_command_sub_ = ROSHandler::instance().nh()->subscribe<std_msgs::String>("/syscommand", 10, std::bind(&APEXRosInterface::command, this, std::placeholders::_1, true));
    private_command_sub_ = ROSHandler::instance().nh()->subscribe<std_msgs::String>("command", 10, std::bind(&APEXRosInterface::command, this, std::placeholders::_1, false));

    ros::spinOnce();
}

void APEXRosInterface::command(const std_msgs::StringConstPtr& cmd, bool global_cmd)
{
    std::string command_string = cmd->data;
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

    if (!disabled_) {
        // disabled state is stronger than pause / unpause
        if (command_string == "pause") {
            core_->setPause(true);

        } else if (command_string == "unpause") {
            core_->setPause(false);
        }
    }

    if (local_cmd) {
        std::size_t parameter_offset = command_string.find_first_of(" ");
        std::string command = command_string.substr(0, parameter_offset);
        std::string parameter_values = command_string.substr(parameter_offset + 1);

        if (command == "stop") {
            disabled_ = true;
            core_->setPause(true);
        } else if (command == "resume") {
            disabled_ = false;
            core_->setPause(false);
        } else if (command == "trigger") {
            GraphImplementationPtr graph = core_->getRoot()->getLocalGraph();

            std::size_t index = parameter_values.find_first_of("/");
            if (index != std::string::npos) {
                std::string node_name = parameter_values.substr(0, index);
                std::string parameter_name = parameter_values.substr(index + 1);

                if (NodeHandle* node_handle = graph->findNodeHandleWithLabel(node_name)) {
                    if (NodePtr node = node_handle->getNode().lock()) {
                        if (node->hasParameter(parameter_name)) {
                            if (auto trigger = node->getParameter<param::TriggerParameter>(parameter_name))
                                trigger->trigger();
                            else
                                std::cerr << "[ROS Command] '" << command_string << "': parameter is not a trigger '" << parameter_name << "'" << std::endl;
                        } else
                            std::cerr << "[ROS Command] '" << command_string << "': unknown parameter '" << parameter_name << "'" << std::endl;
                    }
                } else {
                    std::cerr << "[ROS Command] '" << command_string << "': unknown node '" << node_name << "'" << std::endl;
                }
            }
        }
    }
}

void APEXRosInterface::registerClockWatchdog()
{
    clock_sub_ = ROSHandler::instance().nh()->subscribe<rosgraph_msgs::Clock>("/clock", 10, std::bind(&APEXRosInterface::clock, this, std::placeholders::_1));
}

void APEXRosInterface::clock(const rosgraph_msgs::ClockConstPtr& clock)
{
    ros::Time now = clock->clock;
    if (now < last_clock_ && clock_reset_event_) {
        //        std::cerr << "time reset" << std::endl;

        TokenDataConstPtr data(new connection_types::AnyMessage);
        TokenPtr token = std::make_shared<Token>(data);
        token->setActivityModifier(ActivityModifier::ACTIVATE);
        clock_reset_event_->triggerWith(token);
    }

    last_clock_ = now;
}

void APEXRosInterface::shutdown()
{
    if(ROSHandler::hasInstance()) {
        ROSHandler::instance().stop();
    }

    RosMessageConversion::instance().shutdown();
}
