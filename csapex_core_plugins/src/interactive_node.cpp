/// HEADER
#include <csapex_core_plugins/interactive_node.h>

/// SYSTEM
#include <chrono>

using namespace csapex;

InteractiveNode::InteractiveNode()
    : view_done_(false), stopped_(false)
{
}

void InteractiveNode::process()
{
    view_done_ = false;
}

void InteractiveNode::done()
{
    view_done_ = true;
    wait_for_view_.notify_all();
}

bool InteractiveNode::waitForView()
{
    std::chrono::microseconds poll_time(100);

    std::unique_lock<std::mutex> lock(result_mutex_);
    while(!view_done_) {
        wait_for_view_.wait_for(lock, poll_time);

        if(stopped_) {
            return false;
        }
    }

    return true;
}

void InteractiveNode::abort()
{
    std::unique_lock<std::mutex> lock(result_mutex_);
    stopped_ = true;
    wait_for_view_.notify_all();
}
