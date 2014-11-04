/// HEADER
#include "bag_provider.h"

/// COMPONENT
#include <csapex_ros/ros_message_conversion.h>

/// PROJECT
#include <utils_param/parameter_factory.h>
#include <utils_param/range_parameter.h>

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

    state.addParameter(param::ParameterFactory::declareBool("bag/play", true));
    state.addParameter(param::ParameterFactory::declareBool("bag/loop", true));
    state.addParameter(param::ParameterFactory::declareRange("bag/frame", 0, 1, 0, 1));

    param::Parameter::Ptr topic_param = param::ParameterFactory::declareParameterStringSet("topic",
                                                                                           param::ParameterDescription("topic to play primarily"), set, "");
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
    if(!topic_param_->is<std::string>()) {
        return;
    }
    std::string topic = topic_param_->as<std::string>();

    if(topic == main_topic_) {
        return;
    }

    main_topic_ = topic;

    view_ = new rosbag::View(bag, rosbag::TopicQuery(topic));
    for(rosbag::View::iterator it = view_->begin(); it != view_->end(); ++it) {
        frames_++;
    }
    view_it = view_->begin();
    frames_--;

    param::RangeParameter::Ptr frame = boost::dynamic_pointer_cast<param::RangeParameter>(state.getParameter("bag/frame"));
    frame->setMax(frames_);

    frame_ = 0;

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
    if(!state.readParameter<bool>("bag/play")) {
        if(state.readParameter<int>("bag/frame") == frame_) {
            return false;
        } else {
            // frame was selected, ignore that play is false
        }
    }

    return initiated;
}

connection_types::Message::Ptr BagProvider::next()
{
    connection_types::Message::Ptr r;

    if(!initiated) {
        setTopic();
        return r;
    }

    RosMessageConversion& rmc = RosMessageConversion::instance();

    if(view_it == view_->end()) {
        // loop around?
        if(state.readParameter<bool>("bag/loop")) {
            view_it = view_->begin();
            frame_ = 0;
        }

    } else if(state.readParameter<int>("bag/frame") != frame_) {
        // go to selected frame
        view_it = view_->begin();
        frame_ = state.readParameter<int>("bag/frame");
        std::advance(view_it, frame_);

    } else {
        // advance frame
        view_it++;
        ++frame_;
    }

    if(view_it != view_->end()) {
        rosbag::MessageInstance instance = *view_it;

        r = rmc.instantiate(instance);
        setType(r->toType());
    }

    state["bag/frame"] = frame_;

    return r;
}

Memento::Ptr BagProvider::getState() const
{
    return Memento::Ptr();
}

void BagProvider::setParameterState(Memento::Ptr memento)
{

}
