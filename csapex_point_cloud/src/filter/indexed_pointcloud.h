#ifndef INDEXED_POINTCLOUD_H
#define INDEXED_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class IndexedPointCloud : public Node
{
public:
    IndexedPointCloud();

    virtual void setup();
    virtual void process();

protected:
    ConnectorIn  *input_;
    ConnectorOut *output_;
};
}
#endif // INDEXED_POINTCLOUD_H
