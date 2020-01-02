#ifndef VECTORIZE_PYRAMID_H
#define VECTORIZE_PYRAMID_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class VectorizePyramid : public Node
{
public:
    VectorizePyramid();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex
#endif  // VECTORIZE_PYRAMID_H
