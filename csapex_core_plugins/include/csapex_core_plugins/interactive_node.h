#ifndef INTERACTIVE_NODE_H
#define INTERACTIVE_NODE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/csapex_core_lib_export.h>

/// SYSTEM
#include <mutex>

namespace csapex
{
class CSAPEX_CORE_LIB_EXPORT InteractiveNode : public Node
{
    friend class InteractiveNodeAdapter;

public:
    InteractiveNode();

    void process() final override;
    void process(csapex::NodeModifier& node_modifier, Parameterizable& parameters) final override;

    void process(csapex::NodeModifier& node_modifier, Parameterizable& parameters, Continuation continuation) final override;
    void reset() override;

    bool isAsynchronous() const override;

    void done();

protected:
    virtual void beginProcess(csapex::NodeModifier& node_modifier, Parameterizable& parameters);
    virtual void beginProcess();
    virtual void finishProcess(csapex::NodeModifier& node_modifier, Parameterizable& parameters);
    virtual void finishProcess();

protected:
    bool stopped_;
    bool done_;

    Continuation continuation_;
};

}  // namespace csapex

#endif  // INTERACTIVE_NODE_H
