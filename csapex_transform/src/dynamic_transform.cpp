/// HEADER
#include "dynamic_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_ros/time_stamp_message.h>
#include "listener.h"

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <tf/transform_datatypes.h>

CSAPEX_REGISTER_CLASS(csapex::DynamicTransform, csapex::Node)

using namespace csapex;

DynamicTransform::DynamicTransform()
    : init_(false)
{
}

void DynamicTransform::setupParameters()
{
    std::vector<std::string> topics;
    addParameter(param::ParameterFactory::declareParameterStringSet("from", topics), boost::bind(&DynamicTransform::update, this));
    addParameter(param::ParameterFactory::declareParameterStringSet("to", topics), boost::bind(&DynamicTransform::update, this));

    addParameter(param::ParameterFactory::declareTrigger("refresh"), boost::bind(&DynamicTransform::refresh, this));
    addParameter(param::ParameterFactory::declareTrigger("reset tf"), boost::bind(&DynamicTransform::resetTf, this));

    from_p = boost::dynamic_pointer_cast<param::SetParameter>(getParameter("from"));
    to_p = boost::dynamic_pointer_cast<param::SetParameter>(getParameter("to"));
}

bool DynamicTransform::canTick()
{
    return !frame_in_from_->isConnected() && !frame_in_to_->isConnected() && !time_in_->isConnected();
}

void DynamicTransform::tick()
{
    process();
}

void DynamicTransform::process()
{
    if(!init_) {
        refresh();
    }

    setError(false);
    bool update = false;

    bool use_in_frame = frame_in_from_->hasMessage();
    from_p->setEnabled(!use_in_frame);
    if(use_in_frame) {
        std::string from = frame_in_from_->getValue<std::string>();

        if(readParameter<std::string>("from") != from) {
            setParameter("from", from);
            update = true;
        }
    }


    bool use_to_frame = frame_in_to_->hasMessage();
    to_p->setEnabled(!use_to_frame);
    if(use_to_frame) {
        std::string to = frame_in_to_->getValue<std::string>();

        if(readParameter<std::string>("to") != to) {
            setParameter("to", to);
            update = true;
        }
    }

    if(update) {
        refresh();
    }


    if(time_in_->hasMessage()) {
        connection_types::TimeStampMessage::Ptr time_msg = time_in_->getMessage<connection_types::TimeStampMessage>();
        publishTransform(time_msg->value);
    } else {
        publishTransform(ros::Time(0));
    }
}

void DynamicTransform::publishTransform(const ros::Time& time)
{
    if(!init_) {
        refresh();
    }

    tf::StampedTransform t;

    if(getParameter("from")->is<void>()) {
        throw std::runtime_error("from is not a string");
    }
    if(getParameter("to")->is<void>()) {
        throw std::runtime_error("to is not a string");
    }

    std::string to = readParameter<std::string>("to");
    std::string from = readParameter<std::string>("from");

    try {
        LockedListener l = Listener::getLocked();

        if(l.l) {
            l.l->tfl->lookupTransform(to, from, time, t);
            setError(false);
        } else {
            return;
        }
    } catch(const tf2::TransformException& e) {
        setError(true, e.what(), EL_WARNING);
        return;
    }


    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = t;
    msg->frame_id = from;
    msg->child_frame = to;
    output_->publish(msg);

    output_frame_->publish(readParameter<std::string>("to"));
}

void DynamicTransform::setup()
{
    time_in_ = modifier_->addOptionalInput<connection_types::TimeStampMessage>("Time");
    frame_in_from_ = modifier_->addOptionalInput<std::string>("Origin Frame");
    frame_in_to_ = modifier_->addOptionalInput<std::string>("Target Frame");

    output_ = modifier_->addOutput<connection_types::TransformMessage>("Transform");
    output_frame_ = modifier_->addOutput<std::string>("Target Frame");
}


void DynamicTransform::resetTf()
{
    LockedListener l = Listener::getLocked();
    if(l.l) {
        l.l->reset();
    }
}

void DynamicTransform::refresh()
{
    std::vector<std::string> frames;

    std::string to, from;

    if(getParameter("from")->is<std::string>()) {
        to = to_p->as<std::string>();
    }
    if(getParameter("to")->is<std::string>()) {
        from = from_p->as<std::string>();
    }


    LockedListener l = Listener::getLocked();
    if(!l.l) {
        return;
    }
    // TODO: locks up when no clock is published...
    l.l->tfl->waitForTransform(from, to, ros::Time(0), ros::Duration(1.0));
    if(l.l) {
        std::vector<std::string> f;
        l.l->tfl->getFrameStrings(f);

        for(std::size_t i = 0; i < f.size(); ++i) {
            frames.push_back(std::string("/") + f[i]);
        }

    } else {
        return;
    }

    if(std::find(frames.begin(), frames.end(), from) == frames.end()) {
        frames.push_back(from);
    }
    if(std::find(frames.begin(), frames.end(), to) == frames.end()) {
        frames.push_back(to);
    }

    from_p->setSet(frames);
    to_p->setSet(frames);

    init_ = true;
}

void DynamicTransform::update()
{
}
