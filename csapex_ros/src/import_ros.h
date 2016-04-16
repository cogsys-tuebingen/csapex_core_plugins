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

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void setupROS() override;
    virtual void processROS() override;
    virtual bool canTick() override;
    virtual bool tickROS() override;
    virtual void reset() override;

    void callback(ConnectionTypeConstPtr message);

protected:
    void refresh();
    void update();
    void updateRate();
    void updateSubscriber();
    bool doSetTopic();
    void setTopic(const ros::master::TopicInfo& topic);

    virtual void setParameterState(Memento::Ptr memento) override;

    void publishLatestMessage();
    bool isStampCovered(const ros::Time& stamp);

private:
    Input* input_time_;
    Output* connector_;

    std::recursive_mutex msgs_mtx_;
    std::deque<connection_types::Message::ConstPtr> msgs_;

    ros::Subscriber current_subscriber;

    static const std::string no_topic_;
    ros::master::TopicInfo current_topic_;

    int buffer_size_;
    bool running_;
};

}


#endif // IMPORT_ROS_H
