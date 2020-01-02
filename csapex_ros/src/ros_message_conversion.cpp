/// HEADER
#include <csapex_ros/ros_message_conversion.h>

/// COMPONENT
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex_ros/generic_ros_message.h>

/// PROJECT
#include <csapex/msg/io.h>

/// SYSTEM
#include <topic_tools/shape_shifter.h>

using namespace csapex;

AmbigousRosConversion::AmbigousRosConversion(const std::string& from) : from(from)
{
    std::stringstream ss;
    ss << "ambiguous conversion from " << from;
    error_msg = ss.str();
}

const char* AmbigousRosConversion::what() const noexcept
{
    return error_msg.c_str();
}

RosMessageConversion::RosMessageConversion()
{
}

void RosMessageConversion::shutdown()
{
    ros_types_.clear();
    converters_.clear();
    converters_inv_.clear();
}

bool RosMessageConversion::isTopicTypeRegistered(const ros::master::TopicInfo& topic)
{
    return converters_.find(topic.datatype) != converters_.end();
}

void RosMessageConversion::doRegisterConversion(const std::string& apex_type, const std::string& ros_type, Convertor::Ptr c)
{
    bool found = false;
    for (const Convertor::Ptr& exist : converters_[ros_type]) {
        if (exist->apexType() == c->apexType() && exist->rosType() == c->rosType()) {
            found = true;
            break;
        }
    }
    if (!found) {
        ros_types_.push_back(ros_type);
        converters_[ros_type].push_back(c);
        converters_inv_[apex_type].push_back(c);
    }
}

ros::Subscriber RosMessageConversion::subscribe(const ros::master::TopicInfo& topic, int queue, Callback output, ErrorCallback error_callback, int target_type)
{
    if (isTopicTypeRegistered(topic)) {
        const auto& converters = converters_.at(topic.datatype);
        if (converters.size() == 1) {
            return converters.front()->subscribe(topic, queue, output, error_callback);
        } else {
            throw AmbigousRosConversion(topic.datatype);
        }

    } else {
        std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if (!nh) {
            throw std::runtime_error("no ros connection");
        }

        ros::SubscribeOptions ops;
        ops.template init<topic_tools::ShapeShifter>(topic.name, queue, [output](const boost::shared_ptr<topic_tools::ShapeShifter const>& ros_msg) {
            auto msg = std::make_shared<connection_types::GenericRosMessage>();
            msg->value = shared_ptr_tools::to_std_shared(ros_msg);
            if (msg->value) {
                output(msg);
            }
        });
        ops.transport_hints = ros::TransportHints();
        return nh->subscribe(ops);
    }
}

void RosMessageConversion::write(rosbag::Bag& bag, const connection_types::Message::ConstPtr& message, const std::string& topic, int target_type)
{
    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(message);
    if (gen_ros) {
        ros::Time time;
        if (message->stamp_micro_seconds > 0) {
            time.fromNSec(message->stamp_micro_seconds * 1e3);
        } else {
            time = ros::Time::now();
        }
        bag.write(topic, time, *gen_ros->value);

    } else {
        auto it = converters_inv_.find(message->descriptiveName());
        if (it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot convert type ") + message->descriptiveName());
        }

        const auto& converters = it->second;
        Convertor::Ptr converter;
        if (converters.size() == 1) {
            converter = converters.front();
        } else {
            if (target_type < 0 || target_type >= (int)converters.size()) {
                throw AmbigousRosConversion(message->descriptiveName());
            } else {
                converter = it->second.at(target_type);
            }
        }
        return converter->write(bag, message, topic);
    }
}

std::map<std::string, int> RosMessageConversion::getAvailableRosConversions(const TokenDataConstPtr& type) const
{
    std::map<std::string, int> res;

    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(type);
    if (gen_ros) {
        res["auto"] = 0;

    } else {
        auto it = converters_inv_.find(type->descriptiveName());
        if (it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot convert type ") + type->descriptiveName());
        }

        const auto& converters = it->second;
        for (std::size_t i = 0; i < converters.size(); ++i) {
            const Convertor::Ptr converter = converters[i];
            res.insert(std::make_pair(converter->rosType(), (int)i));
        }
    }
    return res;
}

ros::Publisher RosMessageConversion::advertise(TokenData::ConstPtr type, const std::string& topic, int queue, bool latch, int target_type)
{
    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(type);
    if (gen_ros) {
        return advertiseGenericRos(gen_ros, topic, queue, latch);

    } else {
        auto it = converters_inv_.find(type->descriptiveName());
        if (it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot advertise type ") + type->descriptiveName() + " on topic " + topic);
        }

        const auto& converters = it->second;
        Convertor::Ptr converter;
        if (converters.size() == 1) {
            converter = it->second.front();
        } else {
            if (target_type < 0 || target_type >= (int)converters.size()) {
                throw AmbigousRosConversion(type->descriptiveName());
            } else {
                converter = it->second.at(target_type);
            }
        }
        return converter->advertise(topic, queue, latch);
    }
}

ros::Publisher RosMessageConversion::advertiseGenericRos(const connection_types::GenericRosMessage::ConstPtr& gen_ros, const std::string& topic, int queue, bool latch)
{
    if (!gen_ros->value) {
        throw std::runtime_error("generic ros message is null");
    }

    std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
    if (!nh) {
        throw std::runtime_error("no ros connection");
    }

    return gen_ros->value->advertise(*nh, topic, queue, latch);
}

void RosMessageConversion::publish(ros::Publisher& pub, TokenData::ConstPtr msg, int target_type)
{
    auto gen_ros = std::dynamic_pointer_cast<connection_types::GenericRosMessage const>(msg);
    if (gen_ros) {
        if (!gen_ros->value) {
            throw std::runtime_error("generic ros message is null");
        }
        pub.publish(*gen_ros->value);

    } else {
        auto it = converters_inv_.find(msg->descriptiveName());
        if (it == converters_inv_.end()) {
            throw std::runtime_error(std::string("cannot publish message of type ") + msg->descriptiveName());
        }
        const auto& converters = it->second;
        if (converters.size() == 1) {
            return it->second.front()->publish(pub, msg);
        } else {
            if (target_type < 0 || target_type >= (int)converters.size()) {
                throw AmbigousRosConversion(msg->descriptiveName());
            } else {
                return it->second.at(target_type)->publish(pub, msg);
            }
        }
    }
}

connection_types::Message::Ptr RosMessageConversion::instantiate(const rosbag::MessageInstance& source)
{
    auto it = converters_.find(source.getDataType());
    if (it != converters_.end()) {
        const auto& converters = it->second;
        if (converters.size() == 1) {
            return it->second.front()->instantiate(source);
        } else {
            throw AmbigousRosConversion(source.getDataType());
        }
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
