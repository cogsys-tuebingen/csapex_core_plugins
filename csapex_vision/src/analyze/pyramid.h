#ifndef IMAGE_PYRAMID_H
#define IMAGE_PYRAMID_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class Pyramid : public csapex::Node
{
public:
    Pyramid();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    int out_levels_;
    int out_level_idx_;

    csapex::Output* out_pyr_;
    csapex::Output* out_level_;
    csapex::Input* input_;

    void update();
};
}  // namespace csapex

#endif  // IMAGE_PYRAMID_H
