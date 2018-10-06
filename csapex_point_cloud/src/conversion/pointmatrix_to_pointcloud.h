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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Input* input_;
    Output* output_;
};
}  // namespace csapex
#endif  // POINTMATRIX_TO_POINTCLOUD_H
