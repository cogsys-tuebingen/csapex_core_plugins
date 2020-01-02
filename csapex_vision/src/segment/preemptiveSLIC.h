#ifndef PREEMPTIVESLIC_H
#define PREEMPTIVESLIC_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class PreemptiveSLIC : public csapex::Node
{
public:
    PreemptiveSLIC();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    csapex::Output* output_;
    csapex::Input* input_;
};
}  // namespace csapex

#endif  // PREEMPTIVESLIC_H
