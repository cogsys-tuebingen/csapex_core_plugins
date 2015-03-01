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
    typedef std::shared_ptr<Indices const> IndicesPtr;

public:
    LabelClusteredPointCloud();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input*                        input_;
    Input*                        in_indices_;
    Output*                       output_;

    IndicesPtr cluster_indices;
};
}
#endif // LABEL_POINTCLOUD_H
