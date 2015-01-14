/// HEADER
#include "ros_param.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <ros/master.h>
#include <ros/xmlrpc_manager.h>
#include <ros/this_node.h>


CSAPEX_REGISTER_CLASS(csapex::RosParam, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


RosParam::RosParam()
{
}

void RosParam::setupParameters()
{
    addParameter(param::ParameterFactory::declareText("prefix", ""), std::bind(&RosParam::update, this));
}

void RosParam::setupROS()
{
}

void RosParam::processROS()
{
}

void RosParam::update()
{
    XmlRpc::XmlRpcValue params, result, payload;
    params[0] = ros::this_node::getName();
    if (ros::master::execute("getParamNames", params, result, payload, true)) {
        if(!result.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            aerr << "XmlRpc: Wrong type" << std::endl;
            return;
        }

        std::string name = result[1];
        if(name != "Parameter names") {
            aerr << "XmlRpc: Label wrong" << std::endl;
            return;
        }
        setParameterSetSilence(true);
        removeTemporaryParameters();

        std::string prefix = readParameter<std::string>("prefix");

        XmlRpc::XmlRpcValue data = result[2];
        for(std::size_t i = 0, total = data.size(); i < total; ++i) {
            std::string parameter_name = data[i];

            if(parameter_name.substr(0, prefix.size()) != prefix) {
                continue;
            }

            XmlRpc::XmlRpcValue parameter_value;
            ros::param::getCached(parameter_name, parameter_value);

            std::string label = std::string("parameter") + parameter_name;

            switch(parameter_value.getType()) {
            case XmlRpc::XmlRpcValue::TypeInt: {
                param::Parameter::Ptr p = param::ParameterFactory::declareValue<int>(label, parameter_value);
                addTemporaryParameter(p, std::bind(static_cast<void(*)(const std::string&,int)> (&ros::param::set), parameter_name,
                                                     std::bind(&param::Parameter::as<int>, p.get())));
            }
                break;
            case XmlRpc::XmlRpcValue::TypeDouble: {
                param::Parameter::Ptr p = param::ParameterFactory::declareValue<double>(label, parameter_value);
                addTemporaryParameter(p, std::bind(static_cast<void(*)(const std::string&,double)> (&ros::param::set), parameter_name,
                                                     std::bind(&param::Parameter::as<double>, p.get())));
            }
                break;
            case XmlRpc::XmlRpcValue::TypeBoolean: {
                param::Parameter::Ptr p = param::ParameterFactory::declareBool(label, parameter_value);
                addTemporaryParameter(p, std::bind(static_cast<void(*)(const std::string&,bool)> (&ros::param::set), parameter_name,
                                                     std::bind(&param::Parameter::as<bool>, p.get())));
            }
                break;
            case XmlRpc::XmlRpcValue::TypeString: {
                param::Parameter::Ptr p = param::ParameterFactory::declareText(label, parameter_value);
                addTemporaryParameter(p, std::bind(static_cast<void(*)(const std::string&,const std::string&)> (&ros::param::set), parameter_name,
                                                     std::bind(&param::Parameter::as<std::string>, p.get())));
            }
                break;

            default:
                break;
            }
        }

        setParameterSetSilence(false);
        triggerParameterSetChanged();

        ainfo << "call worked" << std::endl;
    } else {
        aerr << "call failed" << std::endl;
    }
}

