#ifndef OPTIMIZATION_DUMMY_H
#define OPTIMIZATION_DUMMY_H

/// COMPONENT

/// PROJECT
#include <csapex/model/tickable_node.h>

/// SYSTEM

namespace csapex {


class OptimizationDummy : public csapex::TickableNode
{
public:
    OptimizationDummy();

    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    virtual bool canTick() override;

private:
    void start();
    virtual void tick() override;

private:
    Slot* in_;
    Output* out_;

    bool evaluate_;
};


}

#endif // OPTIMIZATION_DUMMY_H
