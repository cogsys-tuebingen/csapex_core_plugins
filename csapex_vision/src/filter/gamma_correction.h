#ifndef GAMMA_CORRECTION_H
#define GAMMA_CORRECTION_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class GammaCorrection : public csapex::Node
{
    enum Type
    {
        POWER_LAW,
        LOGARITHM
    };

public:
    GammaCorrection();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    Input* in_;
    Output* out_;
};

}  // namespace csapex

#endif  // GAMMA_CORRECTION_H
