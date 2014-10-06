#ifndef IMPORT_ROS_H
#define IMPORT_ROS_H

/// COMPONENT
#include <csapex_ros/ros_node.h>

/// PROJECT
#include <csapex/msg/message.h>

/// SYSTEM
#include <deque>

namespace csapex {

class ImportRos : public RosNode
{
public:
    ImportRos();

    virtual void setup();
    virtual void setupParameters();
    virtual void setupROS();
    virtual void processROS();
    virtual void tickROS();
    void abort();

    void callback(ConnectionTypePtr message);

protected:
    void refresh();
    void update();
    void updateRate();
    void updateSubscriber();
    void doSetTopic();
    void setTopic(const ros::master::TopicInfo& topic);

    virtual void setParameterState(Memento::Ptr memento);

    void publishLatestMessage();
    bool isStampCovered(const ros::Time& stamp);

private:
    Input* input_time_;
    Output* connector_;
    std::deque<connection_types::Message::Ptr> msgs_;

    ros::Subscriber current_subscriber;

    static const std::string no_topic_;
    ros::master::TopicInfo current_topic_;

    int retries_;
    ros::WallTime next_retry_;

    bool running_;
};

}


#endif // IMPORT_ROS_H
