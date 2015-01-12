/// HEADER
#include <csapex_ros/ros_handler.h>

/// SYSTEM
#include <QtConcurrentRun>
#include <boost/foreach.hpp>

using namespace csapex;

ROSHandler* ROSHandler::g_instance_ = nullptr;

ROSHandler::ROSHandler(Settings& settings)
    : settings_(settings), initialized_(false)
{
    initHandle(true);
}

ROSHandler::~ROSHandler()
{
    stop();
}

void ROSHandler::stop()
{
    BOOST_FOREACH (const boost::function<void()>& f, shutdown_callbacks_) {
        f();
    }

    if(ros::isStarted()) {
        if(nh_) {
            nh_.reset();
        }
        ros::requestShutdown();
        ros::waitForShutdown();

        if(spinner_) {
            spinner_->stop();
            spinner_.reset();
        }
    }
}

boost::shared_ptr<ros::NodeHandle> ROSHandler::nh()
{
    refresh();
    return nh_;
}

void ROSHandler::initHandle(bool try_only)
{
    if(!initialized_) {
        checkMasterConnection();
    }

    QMutexLocker lock(&has_connection_mutex);
    if(try_only && has_connection.isRunning()) {
        return;
    }

    if(has_connection.result() && !nh_) {
        nh_.reset(new ros::NodeHandle("~"));
        spinner_.reset(new ros::AsyncSpinner(1));
        spinner_->start();

        lock.unlock();

        BOOST_FOREACH (const boost::function<void()>& f, connection_callbacks_) {
            f();
        }
    }
}

bool ROSHandler::isConnected()
{
    QMutexLocker lock(&has_connection_mutex);
    if(has_connection.isRunning()) {
        return false;
    } else {
        return has_connection.result();
    }
}

bool ROSHandler::topicExists(const std::string &topic)
{
    refresh();

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
        if(it->name == topic) {
            return true;
        }
    }

    return false;
}

void ROSHandler::registerConnectionCallback(boost::function<void ()> f)
{
    if(isConnected()) {
        f();
    }
    connection_callbacks_.push_back(f);
}

void ROSHandler::registerShutdownCallback(boost::function<void ()> f)
{

    shutdown_callbacks_.push_back(f);
}


void ROSHandler::checkMasterConnection()
{
    QMutexLocker lock(&has_connection_mutex);

    if(!ros::isInitialized()) {
        std::vector<std::string> additional_args;
        if(settings_.knows("additional_args")) {
            additional_args = settings_.get< std::vector<std::string> >("additional_args");
        }
        int argc = (int) additional_args.size();
        char** argv = (char**) additional_args.data();
        ros::init(argc, argv, "csapex");
    }
    //initialized_ = true;

    if(!has_connection.isRunning()) {
        has_connection = QtConcurrent::run(ros::master::check);
    }
}

void ROSHandler::waitForConnection()
{
    ros::WallRate rate(1);
    while(true) {
        checkMasterConnection();

        QMutexLocker lock(&has_connection_mutex);
        has_connection.waitForFinished();

        if(isConnected()) {
            return;
        } else {
            rate.sleep();
        }
    }
}

void ROSHandler::refresh()
{
    if(!isConnected()) {
        checkMasterConnection();
        has_connection.waitForFinished();
        if(!isConnected()) {
            return;
        }
    }

    if(nh_) {
        QMutexLocker lock(&has_connection_mutex);
        // connection was there
        has_connection.waitForFinished();
        if(!has_connection.result()) {
            // connection no longer there
            stop();
        }
    }

    initHandle();
}
