#ifndef INTERACTIVE_NODE_H
#define INTERACTIVE_NODE_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QMutex>
#include <QWaitCondition>

namespace csapex {

class InteractiveNode : public Node
{
    friend class InteractiveNodeAdapter;

public:
    InteractiveNode();

    virtual void process();
    virtual void stop();
    virtual void setNodeWorker(NodeWorker* nw);

    void done();

protected:
    bool waitForView();

protected:
    QMutex result_mutex_;
    QWaitCondition wait_for_view_;

    bool view_done_;
    bool stopped_;
};

}

#endif // INTERACTIVE_NODE_H
