#ifndef ROS_MESSAGE_CONVERSION_H
#define ROS_MESSAGE_CONVERSION_H

/// COMPONENT
#include <csapex_ros/ros_handler.h>
#include <csapex/msg/generic_pointer_message.hpp>

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <ros/master.h>
#include <ros/subscriber.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <rosbag/view.h>

namespace csapex
{

class Output;


template <typename T>
class RosMessageConversionT;

class Convertor
{
public:
    typedef boost::shared_ptr<Convertor> Ptr;

public:
    virtual ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, Output* output) = 0;
    virtual ros::Publisher advertise(const std::string& topic,  int queue, bool latch = false) = 0;

    virtual void publish(ros::Publisher& pub, ConnectionType::Ptr msg) = 0;
    virtual connection_types::Message::Ptr instantiate(const rosbag::MessageInstance& source) = 0;

    virtual std::string rosType() = 0;
    virtual std::string apexType() = 0;

protected:
    void publish_apex(Output* output, ConnectionType::Ptr msg);
};


template <typename T>
class IdentityConvertor : public Convertor
{
    typedef IdentityConvertor<T> Self;
public:
    std::string rosType() {
        return ros::message_traits::DataType<T>::value();
    }
    std::string apexType() {
        return ros::message_traits::DataType<T>::value();
    }

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, Output* output) {
        boost::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->subscribe<T>(topic.name, queue, boost::bind(&Self::callback, this, output, _1));
    }
    ros::Publisher advertise(const std::string& topic, int queue, bool latch = false) {
        boost::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->advertise<T>(topic, queue, latch);
    }
    void publish(ros::Publisher& pub, ConnectionType::Ptr apex_msg_raw) {
        typename connection_types::GenericPointerMessage<T>::Ptr msg =
                boost::dynamic_pointer_cast<connection_types::GenericPointerMessage<T> > (apex_msg_raw);
        if(!msg) {
            throw std::runtime_error("trying to publish an empty message");
        }
        return pub.publish(msg->value);
    }

    void callback(Output* output, const typename T::ConstPtr& ros_msg) {
        if(!ros_msg) {
            throw std::runtime_error("received an empty ros message");
        }
        typename connection_types::GenericPointerMessage<T>::Ptr apex_msg(new connection_types::GenericPointerMessage<T>);
        apex_msg->value.reset(new T(*ros_msg));
        publish_apex(output, apex_msg);
    }

    connection_types::Message::Ptr instantiate(const rosbag::MessageInstance&) {
        throw std::logic_error("cannot call 'instantiate' on an IdentityConverter");
    }
};

template <typename ROS, typename APEX, typename Converter>
class ConverterTemplate : public Convertor
{
    typedef ConverterTemplate<ROS, APEX, Converter> Self;

public:
    std::string rosType() {
        return ros::message_traits::DataType<ROS>::value();
    }
    std::string apexType() {
        return connection_types::type<APEX>::name();
    }

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, Output* output) {
        boost::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->subscribe<ROS>(topic.name, queue, boost::bind(&Self::callback, this, output, _1));
    }
    ros::Publisher advertise(const std::string& topic, int queue, bool latch = false) {
        boost::shared_ptr<ros::NodeHandle> nh = ROSHandler::instance().nh();
        if(!nh) {
            throw std::runtime_error("no ros connection");
        }

        return nh->advertise<ROS>(topic, queue, latch);
    }
    void publish(ros::Publisher& pub, ConnectionType::Ptr apex_msg_raw) {
        typename APEX::Ptr apex_msg = boost::dynamic_pointer_cast<APEX> (apex_msg_raw);
        if(!apex_msg->isValid()) {
            throw std::runtime_error("trying to publish an empty message");
        }
        typename ROS::Ptr ros_msg = Converter::apex2ros(apex_msg);
        return pub.publish(ros_msg);
    }

    void callback(Output* output, const typename ROS::ConstPtr& ros_msg) {
        if(!ros_msg) {
            throw std::runtime_error("received an empty ros message");
        }
        typename APEX::Ptr apex_msg = Converter::ros2apex(ros_msg);
        publish_apex(output, apex_msg);
    }

    connection_types::Message::Ptr instantiate(const rosbag::MessageInstance& source) {
        return Converter::ros2apex(source.instantiate<ROS>());
    }
};

class RosMessageConversion : public Singleton<RosMessageConversion>
{
    friend class Singleton<RosMessageConversion>;

    template <typename T>
    friend class RosMessageConversionT;

private:
    RosMessageConversion();

public:
    template <typename ROS, typename APEX, typename Converter>
    static void registerConversion() {
        Convertor::Ptr converter(new ConverterTemplate<ROS, APEX, Converter>);
        instance().doRegisterConversion(converter->apexType(), converter->rosType(), converter);
    }

    bool canHandle(const ros::master::TopicInfo &topic);

    ros::Subscriber subscribe(const ros::master::TopicInfo &topic, int queue, Output *output);
    ros::Publisher advertise(ConnectionType::Ptr, const std::string& topic,  int queue, bool latch = false);
    void publish(ros::Publisher& pub, ConnectionType::Ptr msg);

    connection_types::Message::Ptr instantiate(const rosbag::MessageInstance& source);

    std::vector<std::string> getRegisteredRosTypes() {
        return ros_types_;
    }

private:
    void doRegisterConversion(const std::string &apex_type, const std::string &ros_type, Convertor::Ptr c);

    template <typename T>
    bool doCanConvert(typename boost::enable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0) {
        return converters_inv_.find(ros::message_traits::DataType<T>::value()) != converters_inv_.end();
    }
    template <typename T>
    bool doCanConvert(typename boost::disable_if<ros::message_traits::IsMessage<T> >::type* dummy = 0) {
        return false;
    }

private:
    std::vector<std::string> ros_types_;
    std::map<std::string, Convertor::Ptr> converters_;
    std::map<std::string, Convertor::Ptr> converters_inv_;
};


template <typename T>
class RosMessageConversionT
{
public:
    template <typename U>
    static void registerConversionImpl(const std::string& name, typename boost::enable_if<ros::message_traits::IsMessage<U> >::type* dummy = 0) {
        if(!canConvert()) {
            Convertor::Ptr converter(new IdentityConvertor<T>);
            RosMessageConversion::instance().doRegisterConversion(converter->apexType(), converter->rosType(), converter);
            if(!name.empty() && (name != converter->apexType() || name != converter->rosType())) {
                RosMessageConversion::instance().doRegisterConversion(name, name, converter);
            }
        }
    }
    template <typename U>
    static void registerConversionImpl(const std::string& name, typename boost::disable_if<ros::message_traits::IsMessage<U> >::type* dummy = 0) {
    }

    static void registerConversion() {
        registerConversionImpl<T>(type2name(typeid(T)));
    }

    static bool canConvert() {
        return RosMessageConversion::instance().doCanConvert<T>();
    }
};


}

#endif // ROS_MESSAGE_CONVERSION_H
