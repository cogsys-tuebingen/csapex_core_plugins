#ifndef MORPOLOCIAL_H
#define MORPOLOCIAL_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class Morpholocial : public csapex::Node
{
public:
    Morpholocial();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Output* output_;

    Input* input_;
};
}  // namespace csapex

#endif  // MORPOLOCIAL_H
