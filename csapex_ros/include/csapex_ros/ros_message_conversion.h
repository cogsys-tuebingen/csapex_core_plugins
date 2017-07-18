#ifndef ROS_MESSAGE_CONVERSION_H
#define ROS_MESSAGE_CONVERSION_H

/// COMPONENT
#include <csapex_ros/ros_handler.h>
#include <csapex_ros/generic_ros_message.h>
#include <csapex/msg/generic_pointer_message.hpp>

/// PROJECT
#include <csapex/msg/msg_fwd.h>
#include <csapex/msg/message.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/singleton.hpp>
#include <csapex/utility/shared_ptr_tools.hpp>

/// SYSTEM
#include <ros/master.h>
#include <ros/subscriber.h>
#include <memory>
#include <rosbag/view.h>

namespace csapex
{

class Convertor
{
public:
    typedef std::shared_ptr<Convertor> Ptr;
    typedef std::function<void(TokenDataConstPtr)> Callback;

public:
    virtual void write(rosbag::Bag& bag, const connection_types::Message::ConstPtr &source, const std::string& topic) = 0;

    virtual ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, Callback callback) = 0;
    virtual ros::Publisher advertise(const std::string& topic,  int queue, bool latch = false) = 0;

    virtual void publish(ros::Publisher& pub, TokenData::ConstPtr msg) = 0;
    virtual connection_types::Message::Ptr instantiate(const rosbag::MessageInstance& source) = 0;

    virtual std::string rosType() = 0;
    virtual std::string apexType() = 0;

protected:
    void publish_apex(Callback callback, TokenData::ConstPtr msg);
};


template <typename T>
class IdentityConvertor : public Convertor
{
    typedef IdentityConvertor<T> Self;
public:
    std::string rosType() override
    {
        return ros::message_traits::DataType<T>::value();
    }
    std::string apexType() override
    {
        typename connection_types::GenericPointerMessage<T>::Ptr type = std::make_shared<connection_types::GenericPointerMessage<T>>();
        return type->typeName();//ros::message_traits::DataType<T>::value();
    }

    virtual void write(rosbag::Bag& bag, const connection_types::Message::ConstPtr &message, const std::string& topic) override
    {
        auto msg_ptr = std::dynamic_pointer_cast<connection_types::GenericPointerMessage<T> const>(message);
        if(!msg_ptr) {
            throw std::runtime_error("didn't receive a ros message");
        }

        if(message->stamp_micro_seconds > 0) {
            ros::Time time;
            time.fromNSec(message->stamp_micro_seconds * 1e3);
            bag.write(topic, time, *msg_ptr->value);
        }
    }

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, Callback callback) override
    {
        std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->subscribe<T>(topic.name, queue, std::bind(&Self::callback, this, callback, std::placeholders::_1));
    }

    ros::Publisher advertise(const std::string& topic, int queue, bool latch = false) override
    {
        std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->advertise<T>(topic, queue, latch);
    }
    void publish(ros::Publisher& pub, TokenData::ConstPtr apex_msg_raw) override
    {
        typename connection_types::GenericPointerMessage<T>::ConstPtr msg =
                std::dynamic_pointer_cast<connection_types::GenericPointerMessage<T> const> (apex_msg_raw);
        if(!msg) {
            typename connection_types::GenericPointerMessage<T const>::ConstPtr msg =
                    std::dynamic_pointer_cast<connection_types::GenericPointerMessage<T const> const> (apex_msg_raw);
            if(!msg) {
                throw std::runtime_error("cannot cast message to publish");
            }
            auto boost_ptr = shared_ptr_tools::to_boost_shared(msg->value);
            if(!boost_ptr) {
                throw std::runtime_error("cannot convert std shared ptr to boost shared ptr");
            }
            return pub.publish(boost_ptr);
        }
        auto boost_ptr = shared_ptr_tools::to_boost_shared(msg->value);
        if(!boost_ptr) {
            throw std::runtime_error("cannot convert std shared ptr to boost shared ptr");
        }
        return pub.publish(boost_ptr);
    }

    void callback(Callback callback, const typename T::ConstPtr& ros_msg)
    {
        if(!ros_msg) {
            throw std::runtime_error("received an empty ros message");
        }
        typename connection_types::GenericPointerMessage<T>::Ptr apex_msg(new connection_types::GenericPointerMessage<T>);
        apex_msg->value.reset(new T(*ros_msg));
        publish_apex(callback, apex_msg);
    }

    connection_types::Message::Ptr instantiate(const rosbag::MessageInstance& i) override
    {
        typename connection_types::GenericPointerMessage<T>::Ptr res(new connection_types::GenericPointerMessage<T>);
        auto boost_ptr = i.instantiate<T>();
        res->value = shared_ptr_tools::to_std_shared(boost_ptr);
        return res;
    }
};

template <typename ROS, typename APEX, typename Converter>
class ConverterTemplate : public Convertor
{
    typedef ConverterTemplate<ROS, APEX, Converter> Self;

public:
    std::string rosType() override
    {
        return ros::message_traits::DataType<ROS>::value();
    }
    std::string apexType() override
    {
        return connection_types::type<APEX>::name();
    }

    void write(rosbag::Bag& bag, const connection_types::Message::ConstPtr &message, const std::string& topic) override
    {
        std::shared_ptr<const APEX> apex_msg = std::dynamic_pointer_cast<const APEX>(message);
        apex_assert_hard(apex_msg);
        auto ros_msg = Converter::apex2ros(apex_msg);
        if(!ros_msg) {
            throw std::runtime_error("cannot convert apex message to ros message");
        }

        ros::Time time;
        if(message->stamp_micro_seconds > 0) {
            time.fromNSec(message->stamp_micro_seconds * 1e3);
        }
        else{
            time = ros::Time::now();
        }
        bag.write(topic, time, *ros_msg);
    }

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, Callback callback) {
        std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->subscribe<ROS>(topic.name, queue, std::bind(&Self::callback, this, callback, std::placeholders::_1));
    }
    ros::Publisher advertise(const std::string& topic, int queue, bool latch = false) override
    {
        std::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->advertise<ROS>(topic, queue, latch);
    }
    void publish(ros::Publisher& pub, TokenData::ConstPtr apex_msg_raw) override
    {
        typename APEX::ConstPtr apex_msg = std::dynamic_pointer_cast<APEX const> (apex_msg_raw);
        if(!apex_msg->isValid()) {
            throw std::runtime_error("trying to publish an empty message");
        }
        typename ROS::Ptr ros_msg = Converter::apex2ros(apex_msg);
        apex_assert(ros_msg);
        return pub.publish(ros_msg);
    }

    void callback(Callback callback, const typename ROS::ConstPtr& ros_msg)
    {
        if(!ros_msg) {
            throw std::runtime_error("received an empty ros message");
        }
        typename APEX::Ptr apex_msg = Converter::ros2apex(ros_msg);
        apex_assert(apex_msg);
        publish_apex(callback, apex_msg);
    }

    connection_types::Message::Ptr instantiate(const rosbag::MessageInstance& source) {
        return Converter::ros2apex(source.instantiate<ROS>());
    }
};



class AmbigousRosConversion : public std::exception
{
public:
    AmbigousRosConversion(const std::string& from);
    const char* what() const noexcept override;

    const std::string from;
    std::string error_msg;
};


class RosMessageConversion : public Singleton<RosMessageConversion>
{
    friend class Singleton<RosMessageConversion>;

    template <typename T>
    friend class RosMessageConversionT;

public:
    typedef std::function<void(TokenDataConstPtr)> Callback;

private:
    RosMessageConversion();

public:
    template <typename ROS, typename APEX, typename Converter>
    static void registerConversion() {
        Convertor::Ptr converter(new ConverterTemplate<ROS, APEX, Converter>);
        instance().doRegisterConversion(converter->apexType(), converter->rosType(), converter);
    }

    bool isTopicTypeRegistered(const ros::master::TopicInfo &topic);

    void shutdown();

    // outward
    std::map<std::string, int> getAvailableRosConversions(const TokenDataConstPtr& source) const;
    ros::Publisher advertise(TokenData::ConstPtr, const std::string& topic,  int queue, bool latch = false, int target_type = -1);
    void publish(ros::Publisher& pub, TokenData::ConstPtr msg, int target_type = -1);
    void write(rosbag::Bag& bag, const connection_types::Message::ConstPtr &source, const std::string& topic, int target_type = -1);

    // inward
    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, Callback output, int target_type = -1);
    connection_types::Message::Ptr instantiate(const rosbag::MessageInstance& source);

    std::vector<std::string> getRegisteredRosTypes() {
        return ros_types_;
    }

private:
    ros::Publisher advertiseGenericRos(const connection_types::GenericRosMessage::ConstPtr &gen_ros, const std::string& topic,  int queue, bool latch = false);

private:
    void doRegisterConversion(const std::string &apex_type, const std::string &ros_type, Convertor::Ptr c);

    template <typename T>
    bool doCanConvert(typename std::enable_if<ros::message_traits::IsMessage<T>::value >::type* dummy = 0) {
        return converters_inv_.find(ros::message_traits::DataType<T>::value()) != converters_inv_.end();
    }
    template <typename T>
    bool doCanConvert(typename std::enable_if<!ros::message_traits::IsMessage<T>::value >::type* dummy = 0) {
        return false;
    }

private:
    std::vector<std::string> ros_types_;
    std::map<std::string, std::vector<Convertor::Ptr>> converters_;
    std::map<std::string, std::vector<Convertor::Ptr>> converters_inv_;
};


template <typename T>
class RosMessageConversionT
{
public:
    template <typename U>
    static void registerConversionImpl(const std::string& name, typename std::enable_if<ros::message_traits::IsMessage<U>::value >::type* dummy = 0) {
        if(!canConvert()) {
            Convertor::Ptr converter(new IdentityConvertor<T>);
            RosMessageConversion::instance().doRegisterConversion(converter->apexType(), converter->rosType(), converter);
            if(!name.empty() && (name != converter->apexType() || name != converter->rosType())) {
                RosMessageConversion::instance().doRegisterConversion(name, name, converter);
            }
        }
    }
    template <typename U>
    static void registerConversionImpl(const std::string& name, typename std::enable_if<!ros::message_traits::IsMessage<U>::value >::type* dummy = 0) {
    }

    static void registerConversion() {
        registerConversionImpl<T>(type2name(typeid(T)));
    }

    static bool canConvert() {
        return RosMessageConversion::instance().doCanConvert<T>();
    }
};

namespace connection_types
{

template <template <class> class Container, typename T>
class MessageConversionHook<Container, T, typename std::enable_if<ros::message_traits::IsMessage<T>::value>::type>
{
public:
    static void registerConversion() {
        RosMessageConversionT<T>::registerConversion();
    }
};

}

}

#endif // ROS_MESSAGE_CONVERSION_H
