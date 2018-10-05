/// HEADER
#include <csapex_ros/actionlib_node.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>

using namespace csapex;

void ActionlibNodeBase::abortAction()
{
}

void ActionlibNodeBase::startTimer()
{
    startTimer(ros::Duration(0.2));
}

void ActionlibNodeBase::startTimer(ros::Duration max_wait)
{
    timeout_ = getRosHandler().nh()->createTimer(max_wait, [this](const ros::TimerEvent& e) { abortAction(); }, true);
}

bool ActionlibNodeBase::isAsynchronous() const
{
    return true;
}

void ActionlibNodeBase::setupParameters(csapex::Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareTrigger("abort"), [this](param::Parameter*) { abortAction(); });

    parameters.addParameter(param::factory::declareTrigger("detach"), [this](param::Parameter*) { detachAction(); });
}

void ActionlibNodeBase::setup(csapex::NodeModifier& modifier)
{
    RosNode::setup(modifier);

    event_error_ = modifier.addEvent("error");
    event_done_ = modifier.addEvent("done");
}

void ActionlibNodeBase::tearDown()
{
    abortAction();
}

void ActionlibNodeBase::setupROS()
{
    observe(getRosHandler().shutdown, [this]() { tearDown(); });
}

void ActionlibNodeBase::process(csapex::NodeModifier& modifier, csapex::Parameterizable& params, Continuation continuation)
{
    continuation_ = continuation;

    RosNode::process();
}
