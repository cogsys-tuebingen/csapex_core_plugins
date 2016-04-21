#ifndef DELAY_H
#define DELAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

namespace csapex {

class Delay : public Node
{
public:
    Delay();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

    void doSleep();
private:
    Input* input_;
    Output* output_;

    Slot* delayed_slot_;
    Event* delayed_forward_;

    param::OutputProgressParameter* progress_;
};

}

#endif // DELAY_H
