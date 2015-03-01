#ifndef INDEXED_POINTCLOUD_H
#define INDEXED_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class IndexedPointCloud : public Node
{
public:
    IndexedPointCloud();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

protected:
    Input  *input_;
    Output *output_;
};
}
#endif // INDEXED_POINTCLOUD_H
