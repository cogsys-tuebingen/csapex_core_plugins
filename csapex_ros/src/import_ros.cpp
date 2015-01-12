/// HEADER
#include "import_ros.h"

/// COMPONENT
#include <csapex_ros/ros_handler.h>
#include <csapex_ros/ros_message_conversion.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/message_factory.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_worker.h>
#include <csapex_ros/time_stamp_message.h>

/// SYSTEM
#include <yaml-cpp/eventhandler.h>
#include <sensor_msgs/Image.h>
#include <QAction>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>

CSAPEX_REGISTER_CLASS(csapex::ImportRos, csapex::Node)

using namespace csapex;

const std::string ImportRos::no_topic_("-----");

namespace {
ros::Time rosTime(u_int64_t ns) {
    ros::Time t;
    t.fromNSec(ns);
    return t;
}
}

ImportRos::ImportRos()
    : connector_(nullptr), retries_(0), running_(true)
{
}

void ImportRos::setup()
{
    input_time_ = modifier_->addOptionalInput<connection_types::TimeStampMessage>("time");
    connector_ = modifier_->addOutput<connection_types::AnyMessage>("Something");


    boost::function<bool()> connected_condition = (boost::bind(&Input::isConnected, input_time_));

    param::Parameter::Ptr buffer_p = param::ParameterFactory::declareRange("buffer/length", 0.0, 10.0, 1.0, 0.1);
    addConditionalParameter(buffer_p, connected_condition);
    param::Parameter::Ptr max_wait_p = param::ParameterFactory::declareRange("buffer/max_wait", 0.0, 10.0, 1.0, 0.1);
    addConditionalParameter(max_wait_p, connected_condition);
}

void ImportRos::setupParameters()
{
    std::vector<std::string> set;
    set.push_back(no_topic_);
    addParameter(param::ParameterFactory::declareParameterStringSet("topic", set),
                 boost::bind(&ImportRos::update, this));

    addParameter(param::ParameterFactory::declareTrigger("refresh"),
                 boost::bind(&ImportRos::refresh, this));

    addParameter(param::ParameterFactory::declareRange("rate", 0.1, 100.0, 60.0, 0.1),
                 boost::bind(&ImportRos::updateRate, this));
    addParameter(param::ParameterFactory::declareRange("queue", 0, 30, 1, 1),
                 boost::bind(&ImportRos::updateSubscriber, this));
    addParameter(param::ParameterFactory::declareBool("latch", false));
}

void ImportRos::setupROS()
{
    refresh();
}

void ImportRos::refresh()
{
    ROSHandler& rh = getRosHandler();

    if(rh.nh()) {
        std::string old_topic = readParameter<std::string>("topic");

        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        param::SetParameter::Ptr setp = boost::dynamic_pointer_cast<param::SetParameter>(getParameter("topic"));
        if(setp) {
            setError(false);
            bool found = false;
            std::vector<std::string> topics_str;
            topics_str.push_back(no_topic_);
            for(ros::master::V_TopicInfo::const_iterator it = topics.begin(); it != topics.end(); ++it) {
                topics_str.push_back(it->name);
                if(it->name == old_topic) {
                    found = true;
                }
            }
            if(!found) {
                topics_str.push_back(old_topic);
            }
            setp->setSet(topics_str);

            if(old_topic != no_topic_) {
                setp->set(old_topic);
            }
            return;
        }
    }

    setError(true, "no ROS connection", EL_WARNING);
}

void ImportRos::update()
{
    retries_ = 5;
    waitForTopic();
}

void ImportRos::updateRate()
{
    modifier_->setTickFrequency(readParameter<double>("rate"));
}

void ImportRos::updateSubscriber()
{
    if(!current_topic_.name.empty()) {
        current_subscriber = RosMessageConversion::instance().subscribe(current_topic_, readParameter<int>("queue"), boost::bind(&ImportRos::callback, this, _1));
    }
}

bool ImportRos::doSetTopic()
{
    getRosHandler().refresh();

    if(!getRosHandler().isConnected()) {
        setError(true, "no connection to ROS");
        return false;
    }

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    std::string topic = readParameter<std::string>("topic");

    if(topic == no_topic_) {
        return true;
    }

    for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
        if(it->name == topic) {
            setTopic(*it);
            retries_ = 0;
            return true;
        }
    }

    std::stringstream ss;
    ss << "cannot set topic, " << topic << " doesn't exist";
    if(retries_ > 0) {
        ss << ", " << retries_ << " retries left";
    }
    ss << ".";

    setError(true, ss.str(), EL_WARNING);
    return false;
}

void ImportRos::processROS()
{
    // first check if connected -> if not connected, we only use tick
    if(!input_time_->isConnected()) {
        return;
    }

    // now that we are connected, check that we have a valid message
    if(!input_time_->hasMessage()) {
        return;
    }

    // INPUT CONNECTED
    connection_types::TimeStampMessage::ConstPtr time = input_time_->getMessage<connection_types::TimeStampMessage>();

    if(msgs_.empty()) {
        return;
    }

    if(time->value == ros::Time(0)) {
        setError(true, "incoming time is 0, using default behaviour", EL_WARNING);
        publishLatestMessage();
        return;
    }

    if(ros::Time(msgs_.back()->stamp) == ros::Time(0)) {
        setError(true, "buffered time is 0, using default behaviour", EL_WARNING);
        publishLatestMessage();
        return;
    }

    // drop old messages
    ros::Duration keep_duration(readParameter<double>("buffer/length"));
    while(!msgs_.empty() && rosTime(msgs_.front()->stamp) + keep_duration < time->value) {
        msgs_.pop_front();
    }

    if(!msgs_.empty()) {
        if(rosTime(msgs_.front()->stamp) > time->value) {
            // aerr << "time stamp " << time->value << " is too old, oldest buffered is " << rosTime(msgs_.front()->stamp) << std::endl;
            // return;
        }

        ros::Duration max_wait_duration(readParameter<double>("buffer/max_wait"));
        if(rosTime(msgs_.front()->stamp) + max_wait_duration < time->value) {
            // aerr << "[1] time stamp " << time->value << " is too new" << std::endl;
            // return;
        }
    }



    // wait until we have a message
    if(!isStampCovered(time->value)) {
        ros::Rate r(10);
        while(!isStampCovered(time->value) && running_) {
            r.sleep();
            ros::spinOnce();

            if(!msgs_.empty()) {
                ros::Duration max_wait_duration(readParameter<double>("buffer/max_wait"));
                if(rosTime(msgs_.back()->stamp) + max_wait_duration < time->value) {
                    aerr << "[2] time stamp " << time->value << " is too new, latest stamp is " << rosTime(msgs_.back()->stamp) << std::endl;
                    return;
                }
            }
        }
    }

    if(msgs_.empty()) {
        setError(true, "No messages received", EL_WARNING);
        return;
    }

    std::deque<connection_types::Message::ConstPtr>::iterator first_after = msgs_.begin();
    while(rosTime((*first_after)->stamp) < time->value) {
        ++first_after;
    }

    if(first_after == msgs_.begin()) {
        connector_->publish(*first_after);
        return;

    } else if(first_after == msgs_.end()) {
        assert(false);
        setError(true, "Should not happen.....", EL_WARNING);
        return;

    } else {
        std::deque<connection_types::Message::ConstPtr>::iterator last_before = first_after - 1;

        ros::Duration diff1 = rosTime((*first_after)->stamp) - time->value;
        ros::Duration diff2 = rosTime((*last_before)->stamp) - time->value;

        if(diff1 < diff2) {
            connector_->publish(*first_after);
        } else {
            connector_->publish(*last_before);
        }
    }
}

bool ImportRos::isStampCovered(const ros::Time &stamp)
{
    return rosTime(msgs_.back()->stamp) >= stamp;
}

void ImportRos::waitForTopic()
{
    ros::WallDuration poll_wait(0.5);
    while(retries_ --> 0) {
        bool topic_exists = doSetTopic();

        if(topic_exists) {
            return;
        } else {
            ROS_WARN_STREAM("waiting for topic " << readParameter<std::string>("topic"));
            poll_wait.sleep();
        }
    }
}

bool ImportRos::canTick()
{
    return !input_time_->isConnected();
}

void ImportRos::tickROS()
{
    if(retries_ > 0) {
        waitForTopic();
    }

    if(input_time_->isConnected()) {
        return;
    }

    // NO INPUT CONNECTED -> ONLY KEEP CURRENT MESSAGE
    while(msgs_.size() > 1) {
        msgs_.pop_front();
    }

    if(!current_topic_.name.empty()) {
        publishLatestMessage();
    }
}

void ImportRos::publishLatestMessage()
{
    if(msgs_.empty()) {
        ros::WallRate r(10);
        while(msgs_.empty() && running_) {
            r.sleep();
            ros::spinOnce();
        }

        if(!running_) {
            return;
        }
    }

    connector_->publish(msgs_.back());

    if(!readParameter<bool>("latch")) {
        msgs_.clear();
    }
}

void ImportRos::callback(ConnectionTypeConstPtr message)
{
    NodeWorker* nw = getNodeWorker();

    if(!nw) {
        return;
    }

    connection_types::Message::ConstPtr msg = boost::dynamic_pointer_cast<connection_types::Message const>(message);
    if(msg && !nw->isPaused()) {
        if(!msgs_.empty() && msg->stamp < msgs_.front()->stamp) {
            awarn << "detected time anomaly -> reset";
            msgs_.clear();
        }
        msgs_.push_back(msg);
    }
}

void ImportRos::setTopic(const ros::master::TopicInfo &topic)
{
    if(topic.name == current_topic_.name) {
        return;
    }

    current_subscriber.shutdown();

    if(RosMessageConversion::instance().canHandle(topic)) {
        setError(false);

        current_topic_ = topic;
        updateSubscriber();

    } else {
        setError(true, std::string("cannot import topic of type ") + topic.datatype);
        return;
    }

}


void ImportRos::setParameterState(Memento::Ptr memento)
{
    Node::setParameterState(memento);
}

void ImportRos::abort()
{
    running_ = false;
}
