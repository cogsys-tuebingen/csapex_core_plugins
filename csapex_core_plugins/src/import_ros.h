#ifndef IMPORT_ROS_H
#define IMPORT_ROS_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <ros/ros.h>

namespace csapex {

class ImportRos : public Node
{
public:
    ImportRos();

    virtual void setup();
    virtual void process();
    virtual void tick();

    virtual QIcon getIcon() const;

protected:
    void refresh();
    void update();
    void doSetTopic();
    void setTopic(const ros::master::TopicInfo& topic);

    virtual void setState(Memento::Ptr memento);

private:
    ConnectorOut* connector_;

    ros::Subscriber current_subscriber;

    static const std::string no_topic_;
    std::string current_topic_;

    int retries_;
    ros::WallTime next_retry_;
};

}


#endif // IMPORT_ROS_H
