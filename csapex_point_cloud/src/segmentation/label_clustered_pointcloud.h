#ifndef LABEL_POINTCLOUD_H
#define LABEL_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
#include <pcl/filters/filter_indices.h>

namespace csapex {
class LabelClusteredPointCloud : public csapex::Node
{
public:
    typedef std::vector<pcl::PointIndices> Indices;
    typedef boost::shared_ptr<Indices const> IndicesPtr;

public:
    LabelClusteredPointCloud();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*                        input_;
    ConnectorIn*                        in_indices_;
    ConnectorOut*                       output_;

    IndicesPtr cluster_indices;
};
}
#endif // LABEL_POINTCLOUD_H
