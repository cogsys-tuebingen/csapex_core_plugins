/// HEADER
#include "import_ros.h"

/// COMPONENT
#include <csapex_ros/ros_handler.h>
#include <csapex_ros/ros_message_conversion.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/message_factory.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <yaml-cpp/eventhandler.h>
#include <sensor_msgs/Image.h>
#include <QAction>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>

CSAPEX_REGISTER_CLASS(csapex::ImportRos, csapex::Node)

using namespace csapex;

const std::string ImportRos::no_topic_("! Select a topic !");

ImportRos::ImportRos()
    : connector_(NULL), retries_(0)
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

void ImportRos::setup()
{
    connector_ = modifier_->addOutput<connection_types::AnyMessage>("Something");
}

void ImportRos::setupROS()
{
    refresh();
}

void ImportRos::refresh()
{
    getRosHandler().refresh();

    if(getRosHandler().nh()) {
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
    doSetTopic();
}

void ImportRos::updateRate()
{
    getNodeWorker()->setTickFrequency(readParameter<double>("rate"));
}

void ImportRos::updateSubscriber()
{
    current_subscriber = RosMessageConversion::instance().subscribe(current_topic_, readParameter<int>("queue"), boost::bind(&ImportRos::callback, this, _1));
}

void ImportRos::doSetTopic()
{
    getRosHandler().refresh();

    if(!getRosHandler().isConnected()) {
        setError(true, "no connection to ROS");
        return;
    }

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    std::string topic = readParameter<std::string>("topic");

    for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); ++it) {
        if(it->name == topic) {
            setTopic(*it);
            retries_ = 0;
            return;
        }
    }

    std::stringstream ss;
    ss << "cannot set topic, " << topic << " doesn't exist";
    if(retries_ > 0) {
        ss << ", " << retries_ << " retries left";
    }
    ss << ".";

    setError(true, ss.str());
}

void ImportRos::processROS()
{
    // NO INPUT
}

void ImportRos::tickROS()
{
    if(retries_ > 0) {
        if(ros::WallTime::now() > next_retry_) {
            doSetTopic();
            --retries_;

            next_retry_ = ros::WallTime::now() + ros::WallDuration(0.5);
        }
    }

    if(!msg_) {
        ros::Rate r(10);
        while(!msg_) {
            r.sleep();
            ros::spinOnce();
        }
    }

    connector_->publish(msg_);

    if(!readParameter<bool>("latch")) {
        msg_.reset();
    }
}

void ImportRos::callback(ConnectionTypePtr message)
{
    msg_ = message;
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
