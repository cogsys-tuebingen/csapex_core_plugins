#ifndef CLUSTERBOUNDARYMASK_H
#define CLUSTERBOUNDARYMASK_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class ClusterBoundaryMask : public csapex::Node
{
public:
    ClusterBoundaryMask();
    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    csapex::Output* output_;
    csapex::Input* input_;
};
}  // namespace csapex
#endif  // CLUSTERBOUNDARYMASK_H
