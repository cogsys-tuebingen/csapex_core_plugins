#ifndef ROS_HANDLER_H
#define ROS_HANDLER_H

/// PROJECT
#include <csapex/core/settings.h>

/// SYSTEM
#include <ros/ros.h>
#include <QFuture>
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
    void initHandle(bool try_only = false);

    bool isConnected();
    bool topicExists(const std::string& topic);

    void checkMasterConnection();
    void waitForConnection();
    void refresh();

    void registerConnectionCallback(std::function<void()>);
    void registerShutdownCallback(std::function<void()>);

private:
    ROSHandler(Settings& settings);

private:
    static ROSHandler* g_instance_;

    Settings& settings_;

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    bool initialized_;
    std::recursive_mutex has_connection_mutex;
    QFuture<bool> has_connection;

    std::vector<std::function<void()> > connection_callbacks_;
    std::vector<std::function<void()> > shutdown_callbacks_;
};

}

#endif // ROS_HANDLER_H
