#ifndef ACTIONLIB_NODE_H
#define ACTIONLIB_NODE_H

/// COMPONENT
#include <csapex_ros/ros_node.h>
#include <csapex/msg/io.h>
#include <csapex/profiling/interlude.hpp>

/// SYSTEM
#include <actionlib/client/simple_action_client.h>

namespace csapex
{

class ActionlibNodeBase : public RosNode
{
public:
    bool isAsynchronous() const;

    void tearDown() override;

    virtual void process(csapex::NodeModifier&, csapex::Parameterizable&,  Continuation continuation) override;

protected:
    virtual void abortAction() = 0;
    virtual void detachAction() = 0;

    void startTimer();
    void startTimer(ros::Duration max_wait);

    void setupParameters(csapex::Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& modifier) override;
    void setupROS() override;


protected:
    Continuation continuation_;
    ros::Timer timeout_;

    Event* event_error_;
    Event* event_done_;
};

template <class ActionSpec>
class ActionlibNode : public ActionlibNodeBase
{
    ACTION_DEFINITION(ActionSpec);

protected:
    virtual void abortAction() override
    {
        INTERLUDE("abort");

        if(active_client_) {
            active_client_->cancelAllGoals();

//            active_client_.reset();
        }

        if(continuation_) {
            continuation_(ProcessingFunction());
            continuation_ = Continuation();
        }
    }

    virtual void detachAction() override
    {
        INTERLUDE("detach");

        if(active_client_) {
            active_client_->stopTrackingGoal();
//            active_client_.reset();
        }

        if(continuation_) {
            continuation_(ProcessingFunction());
            continuation_ = Continuation();
        }

        msg::trigger(event_done_);
    }

    virtual void resultCallback(const actionlib::SimpleClientGoalState& state, const ResultConstPtr& result)
    {
        INTERLUDE("result callback");

        if(!result) {
            aerr << "Received an empty result message. Did the action server crash?" << std::endl;
            msg::trigger(event_error_);

        } else {
            processResultCallback(state, result);
        }

//        active_client_.reset();
        if(continuation_) {
            continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
            continuation_ = Continuation();
        }
    }

    virtual void processResultCallback(const actionlib::SimpleClientGoalState& state, const ResultConstPtr& result)
    {
    }

    virtual void feedbackCallback(const FeedbackConstPtr&)
    {
    }

    virtual void activeCallback()
    {
        if(timeout_.isValid()) {
            timeout_.stop();
        }
    }

    void processROS() override
    {
    }

    void sendGoal(const Goal& goal)
    {
        INTERLUDE("send goal");
        startTimer();
        active_client_->sendGoal(goal,
                         boost::bind(&ActionlibNode<ActionSpec>::resultCallback, this, _1, _2),
                         boost::bind(&ActionlibNode<ActionSpec>::activeCallback, this),
                         boost::bind(&ActionlibNode<ActionSpec>::feedbackCallback, this, _1));
    }

    void sendGoalAndWait(const Goal& goal,
                         ros::Duration execution_timeout = ros::Duration(0,0),
                         ros::Duration preempt_timeout = ros::Duration(0,0))
    {
        INTERLUDE("send goal & wait");
        active_client_->sendGoalAndWait(goal, execution_timeout, preempt_timeout);
//        active_client_.reset();
    }

    void setupClient(const std::string& topic, bool spin_thread)
    {
        active_client_ = std::make_shared<actionlib::SimpleActionClient<ActionSpec>>(topic, spin_thread);
    }


protected:
    std::shared_ptr<actionlib::SimpleActionClient<ActionSpec>> active_client_;
};

template <class ActionSpec>
class ChanneledActionlibNode : public ActionlibNode<ActionSpec>
{
    ACTION_DEFINITION(ActionSpec);

    using Parent = ActionlibNode<ActionSpec>;
    using Parent::sendGoal;
    using Parent::active_client_;
    using Parent::ainfo;
    using Parent::awarn;
    using Parent::aerr;
    using Parent::event_error_;

protected:
    void processROS()
    {
        std::string channel = getChannel();
        if(active_client_) {
            active_client_->cancelAllGoals();
            active_client_.reset();
        }
        active_client_ = std::make_shared<actionlib::SimpleActionClient<ActionSpec>>(channel, true);

        if(!active_client_->isServerConnected()) {
            awarn << "waiting for action server channel " << channel << std::endl;
            if(!active_client_->waitForServer(ros::Duration(1.0))) {
                aerr << "unknown action server channel " + channel << std::endl;;
                msg::trigger(event_error_);
                return;
            }
        }

        Goal goal_msg;
        getGoal(goal_msg);
        sendGoal(goal_msg);
    }

    virtual std::string getChannel() const = 0;
    virtual void getGoal(Goal& goal) = 0;

protected:
//    std::map<std::string, std::shared_ptr<actionlib::SimpleActionClient<ActionSpec>>> clients_;
};

}

#endif // ACTIONLIB_NODE_H
