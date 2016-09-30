/// HEADER
#include <csapex_ros/ros_message_conversion.h>

/// COMPONENT
#include <csapex_ros/generic_ros_message.h>
#include <csapex/msg/generic_pointer_message.hpp>

/// PROJECT
#include <csapex/msg/io.h>

/// SYSTEM
#include <topic_tools/shape_shifter.h>

using namespace csapex;

RosMessageConversion::RosMessageConversion()
{
}

void RosMessageConversion::shutdown()
{
    ros_types_.clear();
    converters_.clear();
    converters_inv_.clear();
}

bool RosMessageConversion::isTopicTypeRegistered(const ros::master::TopicInfo &topic)
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
    if(isTopicTypeRegistered(topic)) {
        return converters_.at(topic.datatype)->subscribe(topic, queue, output);

    } else {
        std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        ros::SubscribeOptions ops;
        ops.template init<topic_tools::ShapeShifter>(topic.name, queue, [output](const boost::shared_ptr<topic_tools::ShapeShifter const>& ros_msg) {
            auto msg = std::make_shared<connection_types::GenericRosMessage>();
            msg->value = shared_ptr_tools::to_std_shared(ros_msg);
            if(msg->value) {
                output(msg);
            }
        });
        ops.transport_hints = ros::TransportHints();
        return nh->subscribe(ops);
    }
}

std::shared_ptr<topic_tools::ShapeShifter const> RosMessageConversion::convert(const connection_types::Message::ConstPtr &message)
{
    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(message);
    if(gen_ros) {
        return gen_ros->value;
    } else {
        std::map<std::string, Convertor::Ptr>::iterator it = converters_inv_.find(message->descriptiveName());
        if(it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot convert type ") + message->descriptiveName());
        }
        Convertor::Ptr converter = it->second;
        return converter->convert(message);
    }
}

ros::Publisher RosMessageConversion::advertise(TokenData::ConstPtr type, const std::string &topic, int queue, bool latch)
{
    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(type);
    if(gen_ros) {
        return advertiseGenericRos(gen_ros, topic, queue, latch);

    } else {
        std::map<std::string, Convertor::Ptr>::iterator it = converters_inv_.find(type->descriptiveName());
        if(it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot advertise type ") + type->descriptiveName() + " on topic " + topic);
        }
        return it->second->advertise(topic, queue, latch);
    }
}

ros::Publisher RosMessageConversion::advertiseGenericRos(const connection_types::GenericRosMessage::ConstPtr& gen_ros, const std::string &topic, int queue, bool latch)
{
    if(!gen_ros->value) {
        throw std::runtime_error("generic ros message is null");
    }

    std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
    if(!nh) {
        throw std::runtime_error("no ros connection");
    }

    return gen_ros->value->advertise(*nh, topic, queue, latch);
}

void RosMessageConversion::publish(ros::Publisher &pub, TokenData::ConstPtr msg)
{

    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(msg);
    if(gen_ros) {
        if(!gen_ros->value) {
            throw std::runtime_error("generic ros message is null");
        }
        pub.publish(*gen_ros->value);

    } else {
        std::map<std::string, Convertor::Ptr>::iterator it = converters_inv_.find(msg->descriptiveName());
        if(it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot publish message of type ") + msg->descriptiveName());
        }
        it->second->publish(pub, msg);
    }
}

connection_types::Message::Ptr RosMessageConversion::instantiate(const rosbag::MessageInstance &source)
{
    auto pos = converters_.find(source.getDataType());
    if(pos != converters_.end()) {
        return pos->second->instantiate(source);
    } else {
        auto msg = std::make_shared<connection_types::GenericRosMessage>();
        auto ros_msg = source.instantiate<topic_tools::ShapeShifter>();
        msg->value = shared_ptr_tools::to_std_shared(ros_msg);

        return msg;
    }
}

void Convertor::publish_apex(Callback callback, TokenData::ConstPtr msg)
{
    callback(msg);
}
