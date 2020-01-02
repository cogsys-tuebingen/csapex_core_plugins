#ifndef NORMALIZE_H
#define NORMALIZE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class Normalize : public csapex::Node
{
public:
    Normalize();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;
    csapex::Input* mask_;
};
}  // namespace csapex

#endif  // NORMALIZE_H
