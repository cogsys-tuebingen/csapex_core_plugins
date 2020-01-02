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

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Output* output_;

    Input* input_;
};

}  // namespace csapex

#endif  // ADAPTIVE_THRESHOLD_H
