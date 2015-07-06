#ifndef ROS_HANDLER_H
#define ROS_HANDLER_H

/// PROJECT
#include <csapex/core/settings.h>

/// SYSTEM
#include <ros/ros.h>
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
        assert(!g_instance_);
        g_instance_ = new ROSHandler(settings);
    }

public:
    ~ROSHandler();
    void stop();

    std::shared_ptr<ros::NodeHandle> nh();

    bool isConnected();
    bool topicExists(const std::string& topic);

    void waitForConnection();
    void refresh();

    void registerConnectionCallback(std::function<void()>);
    void registerShutdownCallback(std::function<void()>);

private:
    ROSHandler(Settings& settings);

    void initHandle(bool try_only = false);
    void checkMasterConnection();

    void waitForCheck();

private:
    static ROSHandler* g_instance_;

    Settings& settings_;

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    bool initialized_;
    std::recursive_mutex has_connection_mutex;
    bool has_connection;

    bool check_is_running;
    std::condition_variable_any check_is_done;

    std::vector<std::function<void()> > connection_callbacks_;
    std::vector<std::function<void()> > shutdown_callbacks_;
};

}

#endif // ROS_HANDLER_H
