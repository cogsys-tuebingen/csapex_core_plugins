#ifndef TOGGLE_H
#define TOGGLE_H

/// COMPONENT
#include <csapex/model/tickable_node.h>

namespace csapex {

namespace boolean {

class Toggle : public TickableNode
{
public:
    Toggle();

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;
    virtual void tick() override;

private:
    void setSignal();

private:
    Output* out;
    bool signal_;
};

}

}

#endif // TOGGLE_H
