#ifndef ROS_HANDLER_H
#define ROS_HANDLER_H

/// PROJECT
#include <csapex/core/settings.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <ros/ros.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

#include <future>
#include <mutex>

namespace csapex
{
class ROSHandler : public boost::noncopyable
{
public:
    static ROSHandler& instance()
    {
        assert(g_instance_);
        return *g_instance_;
    }
    static void createInstance(Settings& settings)
    {
        if (!g_instance_) {
            g_instance_ = new ROSHandler(settings);
        }
    }

public:
    ~ROSHandler();
    void stop();

    std::shared_ptr<ros::NodeHandle> nh();

    bool isConnected();
    bool topicExists(const std::string& topic);

    void waitForConnection();
    void waitForTopic(const std::string& name, std::function<void()> callback);

public:
    slim_signal::Signal<void()> connected;
    slim_signal::Signal<void()> connection_lost;
    slim_signal::Signal<void()> shutdown;

private:
    ROSHandler(Settings& settings);

    void init();

    void waitForInitializatedHandle();
    void checkMasterConnection();

    void doWaitForTopic(const std::string& name, std::function<void()> callback);

    void connectionEstablished();
    void connectionLost();

private:
    static ROSHandler* g_instance_;

    Settings& settings_;

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    bool initialized_;
    std::recursive_mutex has_connection_mutex;
    bool has_connection;

    std::thread connection_thread_;

    bool check_is_running;
    std::condition_variable_any connection_established;

    std::map<std::string, ros::Subscriber> topic_subscribers_;
    std::map<std::string, std::function<void()>> topic_callbacks_;
};

}  // namespace csapex

#endif  // ROS_HANDLER_H
