#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class VoxelGrid : public Node
{
public:
    VoxelGrid();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_cloud_;
    ConnectorOut* output_;
};

}

#endif // VOXEL_GRID_H
