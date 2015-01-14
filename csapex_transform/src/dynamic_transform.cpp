/// HEADER
#include "dynamic_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex_ros/tf_listener.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/signal/trigger.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#include <tf/transform_datatypes.h>

CSAPEX_REGISTER_CLASS(csapex::DynamicTransform, csapex::Node)

using namespace csapex;

DynamicTransform::DynamicTransform()
    : init_(false), initial_retries_(10)
{
}

void DynamicTransform::setupParameters()
{
    std::vector<std::string> topics;
    addParameter(param::ParameterFactory::declareParameterStringSet("source", topics), std::bind(&DynamicTransform::update, this));
    addParameter(param::ParameterFactory::declareParameterStringSet("target", topics), std::bind(&DynamicTransform::update, this));

    addParameter(param::ParameterFactory::declareTrigger("refresh"), std::bind(&DynamicTransform::refresh, this));
    addParameter(param::ParameterFactory::declareTrigger("reset tf"), std::bind(&DynamicTransform::resetTf, this));

    source_p = std::dynamic_pointer_cast<param::SetParameter>(getParameter("source"));
    target_p = std::dynamic_pointer_cast<param::SetParameter>(getParameter("target"));
}

bool DynamicTransform::canTick()
{
    return source_p->noParameters() != 0 && target_p->noParameters() != 0 &&
            !frame_in_source_->isConnected() && !frame_in_target_->isConnected() && !time_in_->isConnected();
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

    bool use_in_frame = frame_in_source_->hasMessage();
    source_p->setEnabled(!use_in_frame);

    if(use_in_frame) {
        std::string from = frame_in_source_->getValue<std::string>();

        if(readParameter<std::string>("source") != from) {
            setParameter("source", from);
            update = true;
        }
    }


    bool use_to_frame = frame_in_target_->hasMessage();
    target_p->setEnabled(!use_to_frame);

    if(use_to_frame) {
        std::string target = frame_in_target_->getValue<std::string>();

        if(readParameter<std::string>("target") != target) {
            setParameter("target", target);
            update = true;
        }
    }

    if(update) {
        refresh();
    }


    if(time_in_->isConnected() && time_in_->hasMessage()) {
        connection_types::TimeStampMessage::ConstPtr time_msg = time_in_->getMessage<connection_types::TimeStampMessage>();
        publishTransform(time_msg->value);
    } else {
        publishTransform(ros::Time(0));
    }
}

void DynamicTransform::publishTransform(const ros::Time& time)
{
    if(!init_ || source_p->noParameters() == 0 || target_p->noParameters() == 0) {
        refresh();
    }

    tf::StampedTransform t;

    if(getParameter("source")->is<void>()) {
        throw std::runtime_error("from is not a string");
    }
    if(getParameter("target")->is<void>()) {
        throw std::runtime_error("to is not a string");
    }

    std::string target = readParameter<std::string>("target");
    std::string source = readParameter<std::string>("source");

    try {
        LockedTFListener l = TFListener::getLocked();

        if(l.l) {
            tf::TransformListener& tfl = *l.l->tfl;
            if(tfl.canTransform(target, source, time)) {
                tfl.lookupTransform(target, source, time, t);
            } else {
                setError(true, "cannot transform...", EL_WARNING);
                return;
            }
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
    msg->frame_id = target;
    msg->child_frame = source;
    output_->publish(msg);

    output_frame_->publish(readParameter<std::string>("target"));
}

void DynamicTransform::setup()
{
    time_in_ = modifier_->addOptionalInput<connection_types::TimeStampMessage>("Time");
    frame_in_source_ = modifier_->addOptionalInput<std::string>("Origin Frame");
    frame_in_target_ = modifier_->addOptionalInput<std::string>("Target Frame");

    output_ = modifier_->addOutput<connection_types::TransformMessage>("Transform");
    output_frame_ = modifier_->addOutput<std::string>("Target Frame");

    modifier_->addSlot("reset", std::bind(&DynamicTransform::resetTf, this));
    reset_ = modifier_->addTrigger("reset");
}


void DynamicTransform::resetTf()
{
    {
        LockedTFListener l = TFListener::getLocked();
        if(l.l) {
            l.l->reset();
        }
    }

    reset_->trigger();
}

void DynamicTransform::refresh()
{
    std::vector<std::string> frames;

    std::string to, from;

    if(getParameter("source")->is<std::string>()) {
        to = target_p->as<std::string>();
    }
    if(getParameter("target")->is<std::string>()) {
        from = source_p->as<std::string>();
    }


    LockedTFListener l = TFListener::getLocked();
    if(!l.l) {
        return;
    }

    if(l.l) {
        std::vector<std::string> f;
        l.l->tfl->getFrameStrings(f);

        bool has_from = false;
        bool has_to = false;
        for(std::size_t i = 0; i < f.size(); ++i) {
            std::string frame = std::string("/") + f[i];
            frames.push_back(frame);

            if(frame == from) {
                has_from = true;
            }
            if(frame == to) {
                has_to = true;
            }
        }

        if(!has_from || !has_to) {
            if(initial_retries_ --> 0) {
                return;
            }
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

    source_p->setSet(frames);
    target_p->setSet(frames);

    init_ = true;
}

void DynamicTransform::update()
{
}
