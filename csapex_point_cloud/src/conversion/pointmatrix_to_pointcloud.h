#ifndef POINTMATRIX_TO_POINTCLOUD_H
#define POINTMATRIX_TO_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class PointmatrixToPointcloud : public csapex::Node
{
public:
    PointmatrixToPointcloud();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Input* input_;
    Output* output_;
};
}  // namespace csapex
#endif  // POINTMATRIX_TO_POINTCLOUD_H
