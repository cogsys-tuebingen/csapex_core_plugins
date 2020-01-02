#ifndef FLIP_H
#define FLIP_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class Flip : public csapex::Node
{
public:
    Flip();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;

    int mode_;
};
}  // namespace csapex

#endif  // FLIP_H
