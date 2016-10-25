#ifndef DELAY_H
#define DELAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/token_data.h>

/// SYSTEM
#include <future>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN Delay : public Node
{
public:
    Delay();
    ~Delay();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters,
                         Continuation continuation);

    virtual void tearDown() override;

    virtual bool isAsynchronous() const;

private:
    void doSleep();

    void delayInput(Continuation continuation);
    void delayEvent();

private:
    Input* input_;
    Output* output_;

    Slot* delayed_slot_;
    Event* delayed_forward_;

    bool blocking_;

    param::OutputProgressParameter* progress_;

    std::future<void> future;

};

}

#endif // DELAY_H
