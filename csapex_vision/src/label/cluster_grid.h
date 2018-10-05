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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Output* output_;
    csapex::Input* input_;
};
}  // namespace csapex

#endif  // CLUSTERGRID_H
