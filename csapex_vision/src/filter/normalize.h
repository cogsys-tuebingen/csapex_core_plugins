#ifndef NORMALIZE_H
#define NORMALIZE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class Normalize : public csapex::Node
{
public:
    Normalize();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Output *output_;
    csapex::Input  *input_;
    csapex::Input  *mask_;
};
}

#endif // NORMALIZE_H
