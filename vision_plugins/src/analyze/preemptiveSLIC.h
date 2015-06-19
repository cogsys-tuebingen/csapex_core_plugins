#ifndef PREEMPTIVESLIC_H
#define PREEMPTIVESLIC_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class PreemptiveSLIC : public csapex::Node
{
public:
    PreemptiveSLIC();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
private:
    csapex::Output*            output_;
    csapex::Input*             input_;

};
}

#endif // PREEMPTIVESLIC_H
