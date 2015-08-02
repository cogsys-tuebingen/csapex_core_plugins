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
void InteractiveNode::process(Parameterizable &parameters)
{
    throw std::runtime_error("not implemented");
}

void InteractiveNode::process(Parameterizable &parameters, std::function<void (std::function<void ()>)> continuation)
{
    continuation_ = continuation;

    done_ = false;
    beginProcess();
}

bool InteractiveNode::isAsynchronous() const
{
    return true;
}

void InteractiveNode::done()
{
    if(!done_){
        done_ = true;
        continuation_([this](){ finishProcess(); });
    }
}

void InteractiveNode::abort()
{
    stopped_ = true;

    if(continuation_) {
        continuation_([](){});
    }
}
