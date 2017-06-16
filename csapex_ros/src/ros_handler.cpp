/// HEADER
#include <csapex_ros/ros_handler.h>

/// SYSTEM
#include <topic_tools/shape_shifter.h>
#include <thread>
#include <chrono>

using namespace csapex;

ROSHandler* ROSHandler::g_instance_ = nullptr;

ROSHandler::ROSHandler(Settings& settings)
    : settings_(settings), initialized_(false), has_connection(false), check_is_running(false)
{    
    init();
    connection_thread_ = std::thread([this]() {
        checkMasterConnection();
    });
}

ROSHandler::~ROSHandler()
{
    check_is_running = false;

    stop();

    if(connection_thread_.joinable()) {
        connection_thread_.join();
    }

    shutdown();
}

void ROSHandler::stop()
{
    std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
    has_connection = false;

    if(ros::isStarted()) {
        if(nh_) {
            nh_.reset();
        }
        ros::shutdown();

        if(spinner_) {
            spinner_->stop();
            spinner_.reset();
        }
    }

    initialized_ = false;
}

std::shared_ptr<ros::NodeHandle> ROSHandler::nh()
{
    waitForInitializatedHandle();
    return nh_;
}

bool ROSHandler::isConnected()
{
    std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
    return has_connection;
}

bool ROSHandler::topicExists(const std::string &topic)
{
    if(!isConnected()) {
        return false;
    }

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
        if(it->name == topic) {
            return true;
        }
    }

    return false;
}


void ROSHandler::init()
{
    if(!initialized_) {
        //if(!ros::isInitialized()) {
            std::vector<std::string> additional_args;
            if(settings_.knows("additional_args")) {
                additional_args = settings_.get< std::vector<std::string> >("additional_args");
            }
            additional_args.insert(additional_args.begin(), settings_.get< std::string >("path_to_bin"));

            std::vector<char*> cstrings;
            for(const std::string& string : additional_args) {
                cstrings.push_back(const_cast<char*>(string.c_str()));
            }

            int argc = (int) cstrings.size();
            char** argv = (char**) cstrings.data();
            ros::init(argc, argv, "csapex");
       // }
        initialized_ = true;

        std::string host = ros::master::getHost();
        if(host != "localhost") {
            std::cerr << "checking connection to host " << host <<
                         ", startup may block, if the host is not reachable!" << std::endl;
        }
        ros::master::setRetryTimeout(ros::WallDuration(0.25));
    }
}

void ROSHandler::checkMasterConnection()
{
    check_is_running = true;

    while(check_is_running)
    {
        bool c = ros::master::check();
        {
            std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
            if(has_connection != c) {
                has_connection = c;
                if(has_connection) {
                    connectionEstablished();
                } else {
                    connectionLost();
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void ROSHandler::connectionEstablished()
{
    std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);

    if(has_connection) {
        nh_.reset(new ros::NodeHandle("~"));
        spinner_.reset(new ros::AsyncSpinner(1));
        spinner_->start();

        connection_established.notify_all();

        std::cout << "ROS connection established" << std::endl;
        connected();
    }
}

void ROSHandler::connectionLost()
{
    stop();
    init();
}


void ROSHandler::waitForConnection()
{
    std::unique_lock<std::recursive_mutex> lock(has_connection_mutex);
    if(!has_connection) {
        connection_established.wait(lock);
    }
}

void ROSHandler::waitForInitializatedHandle()
{
    if(nh_) {
        return;
    }

    if(!isConnected()) {
        waitForConnection();
    }
}


void ROSHandler::waitForTopic(const std::string& name, std::function<void()> callback)
{
    if(isConnected()) {
        doWaitForTopic(name, callback);
    } else {
        connected.connect([this, name, callback](){
            doWaitForTopic(name, callback);
        });
    }
}
void ROSHandler::doWaitForTopic(const std::string& name, std::function<void()> callback)
{
    if(topicExists(name)) {
        // if the topic already exists -> call back
        callback();

    } else {
        // if the topic doesn't exist, create a 'one-shot' subscription that calls back
        ros::SubscribeOptions ops;
        ops.template init<topic_tools::ShapeShifter>(name, 0, [this, name](const boost::shared_ptr<topic_tools::ShapeShifter const>& ros_msg) {
            auto pos = topic_callbacks_.find(name);
            if(pos != topic_callbacks_.end()) {
                // execute callback
                pos->second();
                // forget the callback and the subscriber (one-shot)
                topic_callbacks_.erase(name);
                topic_subscribers_.erase(name);
            }
        });
        ops.transport_hints = ros::TransportHints();

        topic_callbacks_[name] = callback;
        topic_subscribers_[name] = nh()->subscribe(ops);
    }
}
