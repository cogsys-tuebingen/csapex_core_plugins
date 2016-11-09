/// HEADER
#include <csapex_ros/ros_handler.h>

using namespace csapex;

ROSHandler* ROSHandler::g_instance_ = nullptr;

ROSHandler::ROSHandler(Settings& settings)
    : settings_(settings), initialized_(false), check_is_running(false)
{
    initHandle(true);
}

ROSHandler::~ROSHandler()
{
    stop();
}

void ROSHandler::stop()
{
    {
        std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
        check_is_running = false;
        has_connection = false;

        check_is_done.notify_all();
    }

    shutdown();

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

std::shared_ptr<ros::NodeHandle> ROSHandler::nh()
{
    refresh();
    return nh_;
}

void ROSHandler::initHandle(bool try_only)
{
    if(!initialized_) {
        checkMasterConnection();
    }

    bool make_spinner = false;
    {
        std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
        if(try_only && check_is_running) {
            return;
        }


        make_spinner = has_connection && !nh_;

        if(make_spinner) {
            nh_.reset(new ros::NodeHandle("~"));
            spinner_.reset(new ros::AsyncSpinner(1));
            spinner_->start();
        }
    }
    if(make_spinner) {
        connected();
    }
}

bool ROSHandler::isConnected()
{
    waitForCheck();
    return has_connection;
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

//void ROSHandler::registerConnectionCallback(std::function<void ()> f)
//{
//    if(isConnected()) {
//        f();
//    }
//    connection_callbacks_.push_back(f);
//}

//void ROSHandler::registerShutdownCallback(std::function<void ()> f)
//{

//    shutdown_callbacks_.push_back(f);
//}

void ROSHandler::waitForCheck()
{
    std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
    while(check_is_running) {
        check_is_done.wait(has_connection_mutex);
    }
}


void ROSHandler::checkMasterConnection()
{
    if(!ros::isInitialized()) {
        std::vector<std::string> additional_args;
        if(settings_.knows("additional_args")) {
            additional_args = settings_.get< std::vector<std::string> >("additional_args");
        }
        int argc = (int) additional_args.size();
        char** argv = (char**) additional_args.data();
        ros::init(argc, argv, "csapex");
    }
    initialized_ = true;

    if(!check_is_running) {
        {
            std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
            check_is_running = true;
        }

        std::string host = ros::master::getHost();
        if(host != "localhost") {
            std::cerr << "checking connection to host " << host <<
                         ", startup may block, if the host is not reachable!" << std::endl;
        }

        std::async(std::launch::async, [this]() {
            bool c = ros::master::check();
            {
                std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
                has_connection = c;
                check_is_running = false;
            }
            check_is_done.notify_all();
        });
    }
}

void ROSHandler::waitForConnection()
{
    ros::WallRate rate(1);
    while(true) {
        checkMasterConnection();

        waitForCheck();

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
        waitForCheck();
        if(!isConnected()) {
            return;
        }
    }

    if(nh_) {
        std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
        // connection was there
        waitForCheck();
        if(!has_connection) {
            // connection no longer there
            stop();
        }
    }

    initHandle();
}
