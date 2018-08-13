/// HEADER
#include "bag_provider.h"

/// COMPONENT
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>
#include <csapex_ros/tf_listener.h>

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <sensor_msgs/Image.h>
#include <tf2_msgs/TFMessage.h>
#include <rosgraph_msgs/Clock.h>

CSAPEX_REGISTER_CLASS(csapex::BagProvider, csapex::MessageProvider)

using namespace csapex;

BagProvider::BagProvider()
    : pub_setup_(false), has_last_tf_time_(false), view_all_(nullptr), initiated(false), end_signaled_(false)
{
    std::vector<std::string> set;

    state.addParameter(csapex::param::factory::declareBool("bag/play", true));
    state.addParameter(csapex::param::factory::declareBool("bag/loop", false));
    state.addParameter(csapex::param::factory::declareBool("bag/latch", false));
    state.addParameter(csapex::param::factory::declareRange("bag/frame", 0, 1, 0, 1));
    state.addParameter(csapex::param::factory::declareBool("bag/publish tf", false));
    state.addParameter(csapex::param::factory::declareBool("bag/forward_tf", true));
    state.addParameter(csapex::param::factory::declareBool("bag/publish clock", false));

    csapex::param::Parameter::Ptr topic_param = csapex::param::factory::declareParameterStringSet("topic",
                                                                                           csapex::param::ParameterDescription("topic to play <b>primarily</b>"), set, "");
    topic_param_ = std::dynamic_pointer_cast<param::SetParameter>(topic_param);
    assert(topic_param_);
    state.addParameter(topic_param_);

    setupRosPublisher();
}

BagProvider::~BagProvider()
{
}

void BagProvider::setupRosPublisher()
{
    if(pub_setup_ == false && ROSHandler::instance().isConnected()) {
        pub_tf_ = ROSHandler::instance().nh()->advertise<tf2_msgs::TFMessage>("/tf", 5000, false);
        pub_clock_ = ROSHandler::instance().nh()->advertise<rosgraph_msgs::Clock>("/clock", 500);
        pub_setup_ = true;
    }
}

void BagProvider::load(const std::string& file)
{
    file_ = file;
    std::cout << "loading bag file " << file << ", this can take a while..." << std::endl;
    bag.open(file_);

    view_all_ = new rosbag::View(bag);

    std::set<std::string> topics;
    for(rosbag::View::iterator it = view_all_->begin(); it != view_all_->end(); ++it) {
        rosbag::MessageInstance i = *it;
        if(i.getTopic() != "tf") {
            topics.insert(i.getTopic());
        }
    }


    if(!topics.empty()) {
        topics_.clear();
        topics_.assign(topics.begin(), topics.end());
        std::sort(topics_.begin(), topics_.end());
        topic_param_->setSet(topics_);
        topic_param_->set(topics_[0]);
    }

    view_it_ = view_all_->begin();
    for(std::size_t i = 0; i < topics_.size(); ++i) {
        view_it_map_[topics_[i]] = view_all_->end();
    }

    setSlotCount(topics.size());
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
    std::string main_topic = topic_param_->as<std::string>();

    if(main_topic == main_topic_) {
        return;
    }

    main_topic_ = main_topic;

    frames_ = 0;
    rosbag::View temp(bag, rosbag::TopicQuery(main_topic));
    for(rosbag::View::iterator it = temp.begin(); it != temp.end(); ++it) {
        frames_++;
    }

    int current_frame = state.getParameter("bag/frame")->as<int>();

    param::RangeParameter::Ptr frame = std::dynamic_pointer_cast<param::RangeParameter>(state.getParameter("bag/frame"));
    frame->setMax(frames_);
    frame->set(current_frame);

    frame_ = 0;

    initiated = true;
}

std::vector<std::string> BagProvider::getExtensions() const
{
    return { ".bag" };
}

void BagProvider::restart()
{
    std::cerr << "restart bag file" << std::endl;
    state.getParameter("bag/frame")->set(0);
    state.getParameter("bag/play")->set(true);
}

bool BagProvider::hasNext()
{
    if(!initiated) {
        setTopic();
    }

    bool has_next = false;
    // check if the users wants to scroll in time
    int f = state.readParameter<int>("bag/frame");

    if(f < frames_) {
        has_next = initiated;
    }
    // check if we are at the end
    else if(frame_ == frames_) {
        //if(!end_signaled_) {
//            no_more_messages();
//            end_signaled_ = true;
        //}

        if(!state.readParameter<bool>("bag/loop")) {
            has_next = state.readParameter<bool>("bag/latch");
        }
    }
    // default
    else {
        end_signaled_ = false;
    }

    // check if we are paused
    if(!state.readParameter<bool>("bag/play")) {
        has_next = state.readParameter<bool>("bag/latch");
    } else {
        has_next = initiated;
    }

    return has_next;
}

connection_types::Message::Ptr BagProvider::next(std::size_t slot)
{
    connection_types::Message::Ptr r;

    std::string topic = topics_[slot];

    auto it = view_it_map_.find(topic);
    if(it != view_it_map_.end() && it->second != view_all_->end()) {
        rosbag::MessageInstance instance = *view_it_map_[topic];

        RosMessageConversion& rmc = RosMessageConversion::instance();
        r = rmc.instantiate(instance);
        setType(r->toType());

        //if(topic == main_topic_) {
            if(state.readParameter<bool>("bag/publish clock")) {
                if(!pub_setup_) {
                    setupRosPublisher();
                }
                if(pub_setup_) {
                    rosgraph_msgs::Clock::Ptr clock(new rosgraph_msgs::Clock);
                    clock->clock = instance.getTime();
                    pub_clock_.publish(clock);
                }
            }
        //}
    }

    end_signaled_ = false;

    return r;
}

void BagProvider::prepareNext()
{
    advanceIterators();
}

void BagProvider::advanceIterators()
{
    bool reset = false;
    int skip = 0;
    if(state.readParameter<int>("bag/frame") != frame_) {
        // go to selected frame
        reset = true;
        skip = state.readParameter<int>("bag/frame");
        frame_ = skip;

    } else if(frame_ == frames_ || view_it_map_[main_topic_] == view_all_->end()) {
        // loop around?
        if(state.readParameter<bool>("bag/loop")) {
            reset = true;
            frame_ = 0;
            state["bag/frame"] = frame_;
        }

    } else {
        // advance frame
        if(state.readParameter<bool>("bag/play")) {
            ++frame_;
            state["bag/frame"] = frame_;
        }
    }

    if(reset) {
        view_it_ = view_all_->begin();
        for(std::size_t i = 0; i < topics_.size(); ++i) {
            view_it_map_[topics_[i]] = view_all_->end();
        }
    }


    if(frames_ == frame_ && !state.readParameter<bool>("bag/loop")) {
        state.getParameter("bag/play")->set(false);
    }

    if(frame_ == 0) {
        begin();
    }

    bool done = !reset && !state.readParameter<bool>("bag/play");
    bool publish_tf = state.readParameter<bool>("bag/publish tf");
    bool forward_tf = state.readParameter<bool>("bag/forward_tf");
    bool needs_tf = publish_tf && forward_tf;

    for(; view_it_ != view_all_->end() && !done; ++view_it_) {
        rosbag::MessageInstance next = *view_it_;

        std::string topic = next.getTopic();

        if(needs_tf && (topic == "tf" || topic == "/tf")) {
            if(publish_tf && !pub_setup_) {
                setupRosPublisher();
            }
            tf2_msgs::TFMessage::Ptr tfm = next.instantiate<tf2_msgs::TFMessage>();

            TFListener* listener = TFListener::getLocked().l;
            if(has_last_tf_time_ && tfm->transforms.at(0).header.stamp < last_tf_time_ - ros::Duration(1.0)) {
                listener->reset();
            }
            last_tf_time_ = tfm->transforms.at(0).header.stamp;
            has_last_tf_time_ = true;

            if(forward_tf) {
                listener->addTransforms(*tfm);
            }

            if(publish_tf && pub_setup_) {
                pub_tf_.publish(tfm);
            }
        }

        // copy the iterator
        view_it_map_[topic] = rosbag::View::iterator(view_it_);

        if(topic == main_topic_) {
            if(skip > 0) {
                --skip;
            } else {
                done = true;
            }
        }
    }
}

std::string BagProvider::getLabel(std::size_t slot) const
{
    return topics_[slot];
}

GenericStatePtr BagProvider::getState() const
{
    return GenericStatePtr();
}

void BagProvider::setParameterState(GenericStatePtr memento)
{

}
