/// HEADER
#include <csapex_core_plugins/interactive_node.h>

/// SYSTEM
#include <chrono>

using namespace csapex;

InteractiveNode::InteractiveNode()
    : stopped_(false), done_(false)
{
}
void InteractiveNode::process()
{
    throw std::runtime_error("not implemented");
}
void InteractiveNode::process(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    throw std::runtime_error("not implemented");
}

void InteractiveNode::process(csapex::NodeModifier& node_modifier, Parameterizable &parameters,
                              std::function<void(std::function<void (NodeModifier &, Parameterizable &)>)> continuation)
{
    continuation_ = continuation;

    done_ = false;
    beginProcess(node_modifier, parameters);
}

bool InteractiveNode::isAsynchronous() const
{
    return true;
}

void InteractiveNode::done()
{
    if(!done_){
        done_ = true;
        continuation_([this](csapex::NodeModifier& node_modifier, Parameterizable &parameters){
            finishProcess(node_modifier, parameters);
        });
    }
}

void InteractiveNode::abort()
{
    stopped_ = true;

    if(continuation_) {
        continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
    }
}

void InteractiveNode::beginProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    finishProcess();
}

void InteractiveNode::finishProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    finishProcess();
}

void InteractiveNode::beginProcess()
{

}

void InteractiveNode::finishProcess()
{

}
