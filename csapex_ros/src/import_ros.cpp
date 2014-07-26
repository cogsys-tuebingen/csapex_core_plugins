/// HEADER
#include "import_ros.h"

/// COMPONENT
#include <csapex_ros/ros_handler.h>
#include <csapex_ros/ros_message_conversion.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/msg/message_factory.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils_param/parameter_factory.h>
#include <utils_param/set_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

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
}

void ImportRos::setup()
{

    connector_ = modifier_->addOutput<connection_types::AnyMessage>("Something");

    refresh();
}

void ImportRos::refresh()
{
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

void ImportRos::doSetTopic()
{
    getRosHandler().refresh();

    if(!getRosHandler().isConnected()) {
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

void ImportRos::process()
{
    // NO INPUT
}

void ImportRos::tick()
{
    if(retries_ > 0) {
        if(ros::WallTime::now() > next_retry_) {
            doSetTopic();
            --retries_;

            next_retry_ = ros::WallTime::now() + ros::WallDuration(0.5);
        }
    }
}


void ImportRos::setTopic(const ros::master::TopicInfo &topic)
{
    if(topic.name == current_topic_) {
        return;
    }

    current_subscriber.shutdown();

    if(RosMessageConversion::instance().canHandle(topic)) {
        setError(false);

        current_topic_ = topic.name;
        current_subscriber = RosMessageConversion::instance().subscribe(topic, 100, connector_);

    } else {
        setError(true, std::string("cannot import topic of type ") + topic.datatype);
        return;
    }

}


void ImportRos::setState(Memento::Ptr memento)
{
    Node::setState(memento);
}
