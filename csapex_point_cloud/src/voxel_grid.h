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
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_cloud_;
    Output* output_;
};

}

#endif // VOXEL_GRID_H
