#ifndef EXTREMUM_RENDERER_H
#define EXTREMUM_RENDERER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class MatrixToHeatmap : public csapex::Node
{
public:
    MatrixToHeatmap();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    enum ColorType
    {
        BEZIER,
        PARABOLA
    };

    void update();

    ColorType color_type_;
    csapex::Output* output_;
    csapex::Input* input_;
    csapex::Input* mask_;
};
}  // namespace csapex
#endif  // EXTREMUM_RENDERER_H
