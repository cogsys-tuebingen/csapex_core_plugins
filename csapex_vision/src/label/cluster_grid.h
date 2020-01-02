#ifndef CLUSTERGRID_H
#define CLUSTERGRID_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class ClusterGrid : public csapex::Node
{
public:
    ClusterGrid();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;
};
}  // namespace csapex

#endif  // CLUSTERGRID_H
