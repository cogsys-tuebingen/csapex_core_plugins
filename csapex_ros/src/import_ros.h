#ifndef IMPORT_ROS_H
#define IMPORT_ROS_H

/// COMPONENT
#include <csapex_ros/ros_node.h>

namespace csapex {

class ImportRos : public RosNode
{
public:
    ImportRos();

    virtual void setup();
    virtual void setupROS();
    virtual void processROS();
    virtual void tickROS();

    void callback(ConnectionTypePtr message);

protected:
    void refresh();
    void update();
    void updateRate();
    void updateSubscriber();
    void doSetTopic();
    void setTopic(const ros::master::TopicInfo& topic);

    virtual void setParameterState(Memento::Ptr memento);

private:
    Output* connector_;
    ConnectionTypePtr msg_;

    ros::Subscriber current_subscriber;

    static const std::string no_topic_;
    ros::master::TopicInfo current_topic_;

    int retries_;
    ros::WallTime next_retry_;
};

}


#endif // IMPORT_ROS_H
