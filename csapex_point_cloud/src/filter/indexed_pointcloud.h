#ifndef INDEXED_POINTCLOUD_H
#define INDEXED_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class IndexedPointCloud : public Node
{
public:
    IndexedPointCloud();

    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

protected:
    Input* input_;
    Output* output_;
};
}  // namespace csapex
#endif  // INDEXED_POINTCLOUD_H
