#ifndef BAG_PROVIDER_H
#define BAG_PROVIDER_H

/// COMPONENT
#include <csapex/msg/message_provider.h>

/// PROJECT
#include <csapex/param/set_parameter.h>

/// SYSTEM
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/publisher.h>

namespace csapex
{

class BagProvider : public MessageProvider
{
public:
    BagProvider();
    void load(const std::string& file);

    ~BagProvider();

public:
    virtual bool hasNext();
    virtual connection_types::Message::Ptr next(std::size_t slot);
    virtual std::string getLabel(std::size_t slot) const;

    virtual void restart() override;

    virtual std::vector<std::string> getExtensions() const;

    virtual Memento::Ptr getState() const;
    virtual void setParameterState(Memento::Ptr memento);

    virtual void prepareNext() override;

    void parameterChanged();

private:
    void setupRosPublisher();
    void setTopic();

    void advanceIterators();

private:
    std::string file_;

    int frame_;
    int frames_;

    param::SetParameter::Ptr topic_param_;

    std::vector<std::string> topics_;
    std::string main_topic_;
    rosbag::Bag bag;

    ros::Publisher pub_tf_;
    ros::Publisher pub_clock_;
    bool pub_setup_;

    bool has_last_tf_time_;
    ros::Time last_tf_time_;

    rosbag::View* view_all_;
    rosbag::View::iterator view_it_;
    std::map<std::string, rosbag::View::iterator> view_it_map_;

    bool initiated;
    bool end_signaled_;
};

} /// NAMESPACE


#endif // BAG_PROVIDER_H
