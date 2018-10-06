#ifndef ADAPTIVE_THRESHOLD_H
#define ADAPTIVE_THRESHOLD_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class AdaptiveThreshold : public Node
{
public:
    AdaptiveThreshold();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Output* output_;

    Input* input_;
};

}  // namespace csapex

#endif  // ADAPTIVE_THRESHOLD_H
