#ifndef CLUSTERBOUNDARYMASK_H
#define CLUSTERBOUNDARYMASK_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class ClusterBoundaryMask : public csapex::Node
{
public:
    ClusterBoundaryMask();
    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
private:
    csapex::Output*            output_;
    csapex::Input*             input_;
};
}
#endif // CLUSTERBOUNDARYMASK_H
