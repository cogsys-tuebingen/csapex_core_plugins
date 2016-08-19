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
#include <csapex/model/graph_facade.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node.h>
#include <csapex/signal/event.h>
#include <csapex/msg/no_message.h>
#include <csapex/model/token.h>
#include <csapex/msg/any_message.h>

/// SYSTEM
#include <boost/regex.hpp>
#include <boost/algorithm/string/replace.hpp>
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
    last_clock_ = ros::Time(0);
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

    ROSHandler::instance().registerConnectionCallback([this]() {
        registerCommandListener();
        registerClockWatchdog();
    });

    RosMessageConversion::registerConversion<std_msgs::Bool, connection_types::GenericValueMessage<bool>, ConvertIntegral<std_msgs::Bool, bool> >();
    RosMessageConversion::registerConversion<std_msgs::Int32, connection_types::GenericValueMessage<int>, ConvertIntegral<std_msgs::Int32, int> >();
    RosMessageConversion::registerConversion<std_msgs::Float64, connection_types::GenericValueMessage<double>, ConvertIntegral<std_msgs::Float64, double> >();
    RosMessageConversion::registerConversion<std_msgs::String, connection_types::GenericValueMessage<std::string>, ConvertIntegral<std_msgs::String, std::string> >();

    core_->loaded.connect([this](){
        if(ROSHandler::instance().isConnected()) {
            XmlRpc::XmlRpcValue params, result, payload;
            params[0] = ros::this_node::getName();

            std::string prefix = ros::this_node::getName();

            if (ros::master::execute("getParamNames", params, result, payload, true)) {
                if(result.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                    return;
                }

                std::string name = result[1];
                if(name != "Parameter names") {
                    return;
                }


                XmlRpc::XmlRpcValue data = result[2];
                for(std::size_t i = 0, total = data.size(); i < total; ++i) {
                    std::string parameter_name = data[i];

                    if(parameter_name.substr(0, prefix.size()) != prefix) {
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

void APEXRosInterface::setupGraph(Graph *graph)
{
    clock_reset_event_ = graph->createInternalEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), graph->makeUUID("event_ros_time_reset"), "ros time reset");
}

void APEXRosInterface::loadParameterValue(const std::string& prefix, const std::string& parameter_name, const XmlRpc::XmlRpcValue& parameter_value)
{
    std::string apex_name = parameter_name.substr(prefix.size());

    Graph* graph = core_->getRoot()->getGraph();

    std::vector<std::string> levels;

    std::size_t delim = 0;
    while(delim != std::string::npos) {
        delim = apex_name.find('/');
        std::string subname = apex_name.substr(0, delim);
        if(!subname.empty()) {
            levels.push_back(subname);
        }
        apex_name = apex_name.substr(delim+1);
    }

    if(levels.empty()) {
        return;
    }

    NodeHandle* nh = nullptr;
    std::string param_name;
    for(std::size_t i = 0, n = levels.size(); i < n - 1; ++i) {
        const std::string subname = levels.at(i);
        std::cerr << "searching for " << subname << std::endl;
        nh = graph->findNodeHandleWithLabel(subname);
        if(!nh) {
            std::cerr << "no parameter for " << parameter_name << ", label " << subname << " non-existent" << std::endl;
            return;
        }

        if (NodePtr node = nh->getNode().lock()) {
            std::ostringstream param_name_builder;
            std::copy(std::next(levels.begin(), i + 1), levels.end(), std::ostream_iterator<std::string>(param_name_builder, "/"));
            param_name = param_name_builder.str();
            param_name = param_name.substr(0, param_name.size() - 1);
            std::cerr << "searching for param " << param_name << std::endl;
            if (node->hasParameter(param_name))
                break;
            boost::algorithm::replace_all(param_name, " ", "_");
            std::cerr << "searching for param " << param_name << " (fallback)" << std::endl;
            if (node->hasParameter(param_name))
                break;
        }
        if(i < n - 2) {
            graph = dynamic_cast<Graph*>(nh->getNode().lock().get());
            if(!graph) {
                std::cerr << "no parameter for " << parameter_name << ", child " << subname << " is not a graph" << std::endl;
                return;
            }
        }

    }
    if(!nh) {
        return;
    }

    NodePtr node = nh->getNode().lock();
    if(!node) {
        return;
    }

    if(!node->hasParameter(param_name)) {
        std::cerr << "node " << nh->getUUID() << " doesn't have a parameter called " << param_name << std::endl;
        return;
    }

    param::ParameterPtr p = node->getParameter(param_name);

    switch(parameter_value.getType()) {
    case XmlRpc::XmlRpcValue::TypeInt: {
        int val;
        ros::param::get(parameter_name, val);
        p->set(val);
    }
        break;
    case XmlRpc::XmlRpcValue::TypeDouble: {
        double val;
        ros::param::get(parameter_name, val);
        p->set(val);
    }
        break;
    case XmlRpc::XmlRpcValue::TypeBoolean: {
        bool val;
        ros::param::get(parameter_name, val);
        p->set(val);
    }
        break;
    case XmlRpc::XmlRpcValue::TypeString: {
        std::string val;
        ros::param::get(parameter_name, val);
        p->set(val);
    }
        break;

    default:
        break;
    }
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

void APEXRosInterface::registerClockWatchdog()
{
    clock_sub_ = ROSHandler::instance().nh()->subscribe
            <rosgraph_msgs::Clock>("/clock", 10, std::bind(&APEXRosInterface::clock, this, std::placeholders::_1));
}

void APEXRosInterface::clock(const rosgraph_msgs::ClockConstPtr &clock)
{
    ros::Time now = clock->clock;
    if(now < last_clock_) {
        std::cerr << "time reset" << std::endl;

        TokenDataConstPtr data(new connection_types::AnyMessage);
        TokenPtr token = std::make_shared<Token>(data);
        token->setActive(true);
        clock_reset_event_->triggerWith(token);
    }

    last_clock_ = now;
}

void APEXRosInterface::shutdown()
{
    ROSHandler::instance().stop();

    RosMessageConversion::instance().shutdown();
}
