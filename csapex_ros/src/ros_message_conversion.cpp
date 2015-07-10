/// HEADER
#include <csapex_ros/ros_message_conversion.h>

/// COMPONENT
#include <csapex_ros/generic_ros_message.h>

/// PROJECT
#include <csapex/msg/io.h>

/// SYSTEM
#include <topic_tools/shape_shifter.h>

using namespace csapex;

RosMessageConversion::RosMessageConversion()
{
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

ros::Publisher RosMessageConversion::advertise(ConnectionType::ConstPtr type, const std::string &topic, int queue, bool latch)
{
    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(type);
    if(gen_ros) {
        if(!gen_ros->value) {
            throw std::runtime_error("generic ros message is null");
        }

        std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return gen_ros->value->advertise(*nh, topic, queue, latch);

    } else {
        std::map<std::string, Convertor::Ptr>::iterator it = converters_inv_.find(type->typeName());
        if(it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot advertise type ") + type->descriptiveName() + " on topic " + topic);
        }
        return it->second->advertise(topic, queue, latch);
    }
}

void RosMessageConversion::publish(ros::Publisher &pub, ConnectionType::ConstPtr msg)
{

    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(msg);
    if(gen_ros) {
        if(!gen_ros->value) {
            throw std::runtime_error("generic ros message is null");
        }
        pub.publish(*gen_ros->value);

    } else {
        std::map<std::string, Convertor::Ptr>::iterator it = converters_inv_.find(msg->typeName());
        if(it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot publish message of type ") + msg->descriptiveName());
        }
        it->second->publish(pub, msg);
    }
}

connection_types::Message::Ptr RosMessageConversion::instantiate(const rosbag::MessageInstance &source)
{
    return converters_[source.getDataType()]->instantiate(source);
}

//std::vector<std::string> RosMessageConversion::getRegisteredRosTypes()
//{
//    return ros_types_;
//}

void Convertor::publish_apex(Callback callback, ConnectionType::ConstPtr msg)
{
    callback(msg);
}
