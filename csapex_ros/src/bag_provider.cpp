/// HEADER
#include "bag_provider.h"

/// COMPONENT
#include <csapex_ros/ros_message_conversion.h>

/// PROJECT
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <boost/assign.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <sensor_msgs/Image.h>

CSAPEX_REGISTER_CLASS(csapex::BagProvider, csapex::MessageProvider)

using namespace csapex;

BagProvider::BagProvider()
    : view_(NULL), initiated(false)
{
    std::vector<std::string> set;

    param::Parameter::Ptr topic_param = param::ParameterFactory::declareParameterStringSet("topic",
                                                                                           param::ParameterDescription("topic to play"), set, "");
    topic_param_ = boost::dynamic_pointer_cast<param::SetParameter>(topic_param);
    assert(topic_param_);
    state.addParameter(topic_param_);
}

void BagProvider::load(const std::string& file)
{
    file_ = file;
    bag.open(file_);

    RosMessageConversion& rmc = RosMessageConversion::instance();
    view_ = new rosbag::View(bag, rosbag::TypeQuery(rmc.getRegisteredRosTypes()));

    std::set<std::string> topics;
    for(rosbag::View::iterator it = view_->begin(); it != view_->end(); ++it) {
        rosbag::MessageInstance i = *it;
        topics.insert(i.getTopic());
    }


    if(!topics.empty()) {
        std::vector<std::string> topics_vector(topics.begin(), topics.end());
        std::sort(topics_vector.begin(), topics_vector.end());
        topic_param_->setSet(topics_vector);
        topic_param_->set(topics_vector[0]);
    }
}

void BagProvider::parameterChanged()
{
    setTopic();
}

void BagProvider::setTopic()
{
    std::string topic = topic_param_->as<std::string>();

    view_ = new rosbag::View(bag, rosbag::TopicQuery(topic));
    for(rosbag::View::iterator it = view_->begin(); it != view_->end(); ++it) {
        frames_++;
    }
    view_it = view_->begin();
    frames_--;

    initiated = true;
}

BagProvider::~BagProvider()
{
}

std::vector<std::string> BagProvider::getExtensions() const
{
    return boost::assign::list_of<std::string> (".bag");
}

bool BagProvider::hasNext()
{
    return initiated;
}

connection_types::Message::Ptr BagProvider::next()
{
    connection_types::Message::Ptr r;

    if(!initiated) {
        return r;
    }

    RosMessageConversion& rmc = RosMessageConversion::instance();

    if(view_it != view_->end()) {
        rosbag::MessageInstance instance = *view_it;

        r = rmc.instantiate(instance);
        setType(r->toType());

        view_it++;
    }

    return r;
}

Memento::Ptr BagProvider::getState() const
{
    return Memento::Ptr();
}

void BagProvider::setParameterState(Memento::Ptr memento)
{

}
