/// HEADER
#include <csapex_ros/ros_message_conversion.h>

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
    ros_types_.push_back(ros_type);
    converters_[ros_type] = c;
    converters_inv_[apex_type] = c;
}

ros::Subscriber RosMessageConversion::subscribe(const ros::master::TopicInfo &topic, int queue, Callback output)
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

connection_types::Message::Ptr RosMessageConversion::instantiate(const rosbag::MessageInstance &source)
{
    return converters_[source.getDataType()]->instantiate(source);
}

//std::vector<std::string> RosMessageConversion::getRegisteredRosTypes()
//{
//    return ros_types_;
//}

void Convertor::publish_apex(Callback callback, ConnectionType::Ptr msg)
{
    callback(msg);
}
