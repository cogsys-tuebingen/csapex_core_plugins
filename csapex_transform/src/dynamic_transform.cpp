/// HEADER
#include "dynamic_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex_ros/tf_listener.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/signal/event.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#include <tf/transform_datatypes.h>

CSAPEX_REGISTER_CLASS(csapex::DynamicTransform, csapex::Node)

using namespace csapex;

DynamicTransform::DynamicTransform()
    : init_(false), initial_retries_(10), frozen_(false)
{
}

void DynamicTransform::setupParameters(Parameterizable& parameters)
{
    std::vector<std::string> topics;
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterStringSet("source", topics), std::bind(&DynamicTransform::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterStringSet("target", topics), std::bind(&DynamicTransform::update, this));

    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("refresh"), std::bind(&DynamicTransform::refresh, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("reset tf"), std::bind(&DynamicTransform::resetTf, this));

    source_p = std::dynamic_pointer_cast<param::SetParameter>(getParameter("source"));
    target_p = std::dynamic_pointer_cast<param::SetParameter>(getParameter("target"));

    parameters.addParameter(csapex::param::ParameterFactory::declareBool("exact_time", false), exact_time_);

    auto is_frozen = csapex::param::ParameterFactory::declareBool("freeze_transformation", false);
    parameters.addParameter(is_frozen, [this](param::Parameter* p){
        freeze(p->as<bool>());
    });

    std::function<bool()> freeze_condition = [this]() {
        return frozen_;
    };

    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareAngle("~tf/roll", 0.0), freeze_condition, roll);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareAngle("~tf/pitch", 0.0), freeze_condition, pitch);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareAngle("~tf/yaw", 0.0), freeze_condition, yaw);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("~tf/dx", 0.0), freeze_condition, x);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("~tf/dy", 0.0), freeze_condition, y);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("~tf/dz", 0.0), freeze_condition, z);

    refresh();
}

bool DynamicTransform::canTick()
{
    return true;
}

void DynamicTransform::tick()
{
    if(source_p->noParameters() != 0 && target_p->noParameters() != 0 && !msg::isConnected(time_in_))
    {
        process();
    }
}

void DynamicTransform::freeze(bool frozen)
{
    if(frozen != frozen_) {

        if(frozen) {
            if(boost::optional<tf::Transform> t = last_transform) {
                tf::Transform last = *t;

                auto o = last.getOrigin();
                x = o.x();
                y = o.y();
                z = o.z();

                tf::Matrix3x3(last.getRotation()).getEulerYPR(yaw, pitch, roll);

                setParameter("~tf/roll", roll);
                setParameter("~tf/pitch", pitch);
                setParameter("~tf/yaw", yaw);
                setParameter("~tf/dx", x);
                setParameter("~tf/dy", y);
                setParameter("~tf/dz", z);
            }
        }

        frozen_ = frozen;
    }
}

void DynamicTransform::process()
{
    if(!init_) {
        refresh();
    }

    node_modifier_->setNoError();

    try {
        if(msg::isConnected(time_in_) && msg::hasMessage(time_in_)) {
            connection_types::TimestampMessage::ConstPtr time_msg = msg::getMessage<connection_types::TimestampMessage>(time_in_);
            auto nano = std::chrono::duration_cast<std::chrono::nanoseconds>(time_msg->value.time_since_epoch());
            ros::Time time;
            time.fromNSec(nano.count());
            publishTransform(time);
        } else {
            publishTransform(ros::Time(0));
        }
    } catch(const std::exception& e) {
        aerr << "error: " << e.what() << std::endl;
        node_modifier_->setWarning(e.what());
    }
}

void DynamicTransform::publishTransform(const ros::Time& time)
{
    if(!init_ || source_p->noParameters() == 0 || target_p->noParameters() == 0) {
        refresh();
    }

    if(!init_) {
        return;
    }


    std::string target = readParameter<std::string>("target");
    std::string source = readParameter<std::string>("source");

    if(frozen_) {
        connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
        msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
        msg->frame_id = target;
        msg->child_frame = source;
        msg::publish(output_, msg);

    } else {
        tf::StampedTransform t;
        if(getParameter("source")->is<void>()) {
            throw std::runtime_error("from is not a string");
        }
        if(getParameter("target")->is<void>()) {
            throw std::runtime_error("to is not a string");
        }

        try {
            LockedTFListener l = TFListener::getLocked();

            if(l.l) {
                tf::TransformListener& tfl = *l.l->tfl;
                if(tfl.waitForTransform(target, source, time, ros::Duration(0.1))) {
                    tfl.lookupTransform(target, source, time, t);

                } else if(exact_time_) {
                    node_modifier_->setWarning(std::string("cannot exactly transform between ") +
                                               target + " and " + source);
                    return;

                } else {
                    if(tfl.canTransform(target, source, ros::Time(0))) {
                        node_modifier_->setWarning("cannot transform, using latest transform");
                        tfl.lookupTransform(target, source, ros::Time(0), t);
                    } else {
                        node_modifier_->setWarning(std::string("cannot transform between ") +
                                                   target + " and " + source + " at all...");
                        return;
                    }
                }
                node_modifier_->setNoError();
            } else {
                return;
            }
        } catch(const tf2::TransformException& e) {
            node_modifier_->setWarning(e.what());
            return;
        }

        connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
        msg->value = t;
        msg->frame_id = target;
        msg->child_frame = source;
        msg::publish(output_, msg);

        last_transform = t;
    }
}

void DynamicTransform::setup(NodeModifier& node_modifier)
{
    time_in_ = node_modifier.addOptionalInput<connection_types::TimestampMessage>("Time");

    output_ = node_modifier.addOutput<connection_types::TransformMessage>("Transform");

    node_modifier.addSlot("reset", std::bind(&DynamicTransform::resetTf, this));
    reset_ = node_modifier.addEvent("reset");
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

    refresh();
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

        if((!from.empty() && !has_from) ||
                (!to.empty() && !has_to)) {
            if(initial_retries_ --> 0) {
                ainfo << "retry" << std::endl;
                return;
            }
        }


    } else {
        return;
    }

    if(!from.empty()) {
        if(std::find(frames.begin(), frames.end(), from) == frames.end()) {
            frames.push_back(from);
        }
    }
    if(!to.empty()) {
        if(std::find(frames.begin(), frames.end(), to) == frames.end()) {
            frames.push_back(to);
        }
    }

    source_p->setSet(frames);
    target_p->setSet(frames);

    init_ = true;
}

void DynamicTransform::update()
{
}
