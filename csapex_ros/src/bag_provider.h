#ifndef BAG_PROVIDER_H
#define BAG_PROVIDER_H

/// COMPONENT
#include <csapex/msg/message_provider.h>

/// PROJECT
#include <csapex/param/set_parameter.h>

/// SYSTEM
// clang-format on
#include <csapex/utility/suppress_warnings_start.h>
#include <ros/publisher.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format off

namespace csapex
{
class BagProvider : public MessageProvider
{
public:
    BagProvider();
    void load(const std::string& file) override;

    ~BagProvider();

public:
    bool hasNext() override;
    connection_types::Message::Ptr next(std::size_t slot) override;
    std::string getLabel(std::size_t slot) const override;

    void restart() override;

    std::vector<std::string> getExtensions() const override;

    GenericStatePtr getState() const override;
    void setParameterState(GenericStatePtr memento) override;

    void prepareNext() override;

    void parameterChanged() override;

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

}  // namespace csapex

#endif  // BAG_PROVIDER_H
