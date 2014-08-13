/// HEADER
#include <csapex_ros/ros_message_conversion.h>

/// PROJECT
#include <csapex/msg/output.h>

using namespace csapex;

RosMessageConversion::RosMessageConversion()
{
}

bool RosMessageConversion::canHandle(const ros::master::TopicInfo &topic)
{
    return converters_.find(topic.datatype) != converters_.end();
}

void RosMessageConversion::doRegisterConversion(const std::string& apex_type, const std::string& ros_type, Convertor::Ptr c)
{
    converters_[ros_type] = c;
    converters_inv_[apex_type] = c;
}

ros::Subscriber RosMessageConversion::subscribe(const ros::master::TopicInfo &topic, int queue, Output* output)
{
    return converters_.at(topic.datatype)->subscribe(topic, queue, output);
}

ros::Publisher RosMessageConversion::advertise(ConnectionType::Ptr type, const std::string &topic, int queue, bool latch)
{
    std::map<std::string, Convertor::Ptr>::iterator it = converters_inv_.find(type->name());
    if(it == converters_inv_.end()) {
        throw std::runtime_error(std::string("cannot advertise type ") + type->name() + " on topic " + topic);
    }
    return it->second->advertise(topic, queue, latch);
}

void RosMessageConversion::publish(ros::Publisher &pub, ConnectionType::Ptr msg)
{
    std::map<std::string, Convertor::Ptr>::iterator it = converters_inv_.find(msg->name());
    if(it == converters_inv_.end()) {
        throw std::runtime_error(std::string("cannot publish message of type ") + msg->name());
    }
    it->second->publish(pub, msg);
}



void Convertor::publish_apex(Output *output, ConnectionType::Ptr msg)
{
    output->setType(msg);
    output->publish(msg);
}
