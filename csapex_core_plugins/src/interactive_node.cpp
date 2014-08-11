/// HEADER
#include <csapex_core_plugins/interactive_node.h>

/// PROJECT
#include <csapex/model/node_worker.h>

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
    wait_for_view_.wakeAll();
}

bool InteractiveNode::waitForView()
{
    result_mutex_.lock();
    while(!view_done_ && !stopped_) {
        wait_for_view_.wait(&result_mutex_, 100);
        if(!view_done_) {
            getNodeWorker()->checkParameters();
        }
    }
    result_mutex_.unlock();

    return !stopped_;
}

void InteractiveNode::abort()
{
    stopped_ = true;
    wait_for_view_.wakeAll();
}
