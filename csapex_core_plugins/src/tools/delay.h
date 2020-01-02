#ifndef DELAY_H
#define DELAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/token_data.h>

/// SYSTEM
#include <future>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN Delay : public Node
{
public:
    Delay();
    ~Delay();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters, Continuation continuation) override;

    void tearDown() override;

    bool isAsynchronous() const override;

    void reset() override;

private:
    void doSleep();

    void delayInput(Continuation continuation);
    void delayEvent(const TokenPtr& token);

private:
    Input* input_;
    Output* output_;

    Slot* delayed_slot_;
    Event* delayed_forward_;

    bool blocking_;

    param::OutputProgressParameter* progress_;

    std::future<void> future;
};

}  // namespace csapex

#endif  // DELAY_H
