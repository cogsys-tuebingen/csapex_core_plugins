#ifndef INTERACTIVE_NODE_H
#define INTERACTIVE_NODE_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <mutex>
#include <condition_variable>

namespace csapex {

class InteractiveNode : public Node
{
    friend class InteractiveNodeAdapter;

public:
    InteractiveNode();

    virtual void process();
    virtual void abort();

    void done();

protected:
    bool waitForView();

protected:
    std::mutex result_mutex_;
    std::condition_variable wait_for_view_;

    bool view_done_;
    bool stopped_;
};

}

#endif // INTERACTIVE_NODE_H
