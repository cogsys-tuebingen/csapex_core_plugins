/// HEADER
#include "dynamic_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_transform/time_stamp_message.h>
#include "listener.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <tf/transform_datatypes.h>

CSAPEX_REGISTER_CLASS(csapex::DynamicTransform, csapex::Node)

using namespace csapex;

DynamicTransform::DynamicTransform()
{
    addTag(Tag::get("Transform"));
}

void DynamicTransform::setupParameters()
{
    std::vector<std::string> topics;
    topics.push_back("/");
    addParameter(param::ParameterFactory::declareParameterStringSet("from", topics), boost::bind(&DynamicTransform::update, this));
    addParameter(param::ParameterFactory::declareParameterStringSet("to", topics), boost::bind(&DynamicTransform::update, this));

    addParameter(param::ParameterFactory::declareTrigger("refresh"), boost::bind(&DynamicTransform::refresh, this));
    addParameter(param::ParameterFactory::declareTrigger("reset tf"), boost::bind(&DynamicTransform::resetTf, this));

    from_p = boost::dynamic_pointer_cast<param::SetParameter>(getParameter("from"));
    to_p = boost::dynamic_pointer_cast<param::SetParameter>(getParameter("to"));
}

void DynamicTransform::process()
{
    setError(false);
    bool update = false;

    bool use_in_frame = frame_in_from_->isConnected() && frame_in_from_->hasMessage();
    from_p->setEnabled(!use_in_frame);
    if(use_in_frame) {
        std::string from = frame_in_from_->getMessage<connection_types::GenericValueMessage<std::string> >()->value;

        if(param<std::string>("from") != from) {
            parameter_state_["from"] = from;
            update = true;
        }
    }


    bool use_to_frame = frame_in_to_->isConnected() && frame_in_to_->hasMessage();
    to_p->setEnabled(!use_to_frame);
    if(use_to_frame) {
        std::string to = frame_in_to_->getMessage<connection_types::GenericValueMessage<std::string> >()->value;

        if(param<std::string>("to") != to) {
            parameter_state_["to"] = to;
            update = true;
        }
    }

    if(update) {
        refresh();
    }


    connection_types::TimeStampMessage::Ptr time_msg = time_in_->getMessage<connection_types::TimeStampMessage>();
    publishTransform(time_msg->value);
}

void DynamicTransform::tick()
{
    if(time_in_->isConnected()) {
        return;
    }

    publishTransform(ros::Time(0));
}

void DynamicTransform::publishTransform(const ros::Time& time)
{
    tf::StampedTransform t;

    try {
        LockedListener l = Listener::getLocked();

        if(l.l) {
            l.l->tfl->lookupTransform(param<std::string>("to"), param<std::string>("from"), time, t);
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
    output_->publish(msg);

    connection_types::GenericValueMessage<std::string>::Ptr frame(new connection_types::GenericValueMessage<std::string>);
    frame->value = param<std::string>("to");
    output_frame_->publish(frame);
}

void DynamicTransform::setup()
{
    time_in_ = modifier_->addInput<connection_types::TimeStampMessage>("Time", true);
    frame_in_from_ = modifier_->addInput<connection_types::GenericValueMessage<std::string> >("Origin Frame", true);
    frame_in_to_ = modifier_->addInput<connection_types::GenericValueMessage<std::string> >("Target Frame", true);

    output_ = modifier_->addOutput<connection_types::TransformMessage>("Transform");
    output_frame_ = modifier_->addOutput<connection_types::GenericValueMessage<std::string> >("Target Frame");

    refresh();
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

    LockedListener l = Listener::getLocked();
    if(l.l) {
        std::vector<std::string> f;
        l.l->tfl->getFrameStrings(f);

        for(std::size_t i = 0; i < f.size(); ++i) {
            frames.push_back(std::string("/") + f[i]);
        }

    } else {
        return;
    }

    from_p->setSet(frames);
    to_p->setSet(frames);
}

void DynamicTransform::update()
{
}

