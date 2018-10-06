#ifndef OPTIMIZATION_DUMMY_H
#define OPTIMIZATION_DUMMY_H

/// COMPONENT
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex
{
class OptimizationDummy : public csapex::Node
{
public:
    OptimizationDummy();

    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* in_;
    Output* out_;
    Output* out_grad_;
};

}  // namespace csapex

#endif  // OPTIMIZATION_DUMMY_H
