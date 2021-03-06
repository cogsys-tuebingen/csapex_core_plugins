#ifndef IMPORT_ROS_H
#define IMPORT_ROS_H

/// COMPONENT
#include <csapex_ros/ros_node.h>

/// PROJECT
#include <csapex/msg/message.h>

/// SYSTEM
#include <deque>

namespace csapex
{
class ImportRos : public RosNode
{
public:
    ImportRos();
    ~ImportRos();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void setupROS() override;
    void processROS() override;
    bool canProcess() const override;
    void reset() override;
    void tearDown() override;

    void callback(TokenDataConstPtr message);

protected:
    void processSource();
    void processNotSource();

    void refresh();
    void update();
    void updateSubscriber();
    bool doSetTopic();
    void setTopic(const ros::master::TopicInfo& topic);

    void publishLatestMessage();
    bool isStampCovered(const ros::Time& stamp);

private:
    Input* input_time_;
    Output* connector_;

    mutable std::recursive_mutex msgs_mtx_;
    std::deque<connection_types::Message::ConstPtr> msgs_;

    ros::Subscriber current_subscriber;

    static const std::string no_topic_;
    ros::master::TopicInfo current_topic_;

    int buffer_size_;
    bool running_;
    int mode_;
};
}  // namespace csapex

#endif  // IMPORT_ROS_H
