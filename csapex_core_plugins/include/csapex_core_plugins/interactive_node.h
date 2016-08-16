#ifndef INTERACTIVE_NODE_H
#define INTERACTIVE_NODE_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <mutex>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN InteractiveNode : public Node
{
    friend class InteractiveNodeAdapter;

public:
    InteractiveNode();

    virtual void process() final override;
    virtual void process(csapex::NodeModifier& node_modifier, Parameterizable &parameters) final override;

    virtual void process(csapex::NodeModifier& node_modifier, Parameterizable &parameters,
                         std::function<void(std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation) final override;
    virtual void reset() override;

    virtual bool isAsynchronous() const override;

    void done();

protected:
    virtual void beginProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters);
    virtual void beginProcess();
    virtual void finishProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters);
    virtual void finishProcess();

protected:
    bool stopped_;
    bool done_;

    std::function<void (std::function<void (csapex::NodeModifier& node_modifier, Parameterizable &parameters)>)> continuation_;
};

}

#endif // INTERACTIVE_NODE_H
