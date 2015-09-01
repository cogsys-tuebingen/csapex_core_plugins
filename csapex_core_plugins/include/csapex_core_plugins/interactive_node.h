#ifndef INTERACTIVE_NODE_H
#define INTERACTIVE_NODE_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <mutex>

namespace csapex {

class InteractiveNode : public Node
{
    friend class InteractiveNodeAdapter;

public:
    InteractiveNode();

    virtual void process() final override;
    virtual void process(Parameterizable &parameters) final override;

    virtual void process(Parameterizable &parameters, std::function<void(std::function<void ()>)> continuation) final override;
    virtual void abort();

    virtual bool isAsynchronous() const override;

    void done();

protected:
    virtual void beginProcess() = 0;
    virtual void finishProcess() = 0;

protected:
    bool stopped_;
    bool done_;

    std::function<void (std::function<void ()>)> continuation_;
};

}

#endif // INTERACTIVE_NODE_H
