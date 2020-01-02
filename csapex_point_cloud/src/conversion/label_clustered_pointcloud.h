#ifndef LABEL_POINTCLOUD_H
#define LABEL_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

/// SYSTEM
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif  //__clang__
#include <pcl/filters/filter_indices.h>
#if __clang__
#pragma clang diagnostic pop
#endif  //__clang__

namespace csapex
{
class LabelClusteredPointCloud : public csapex::Node
{
public:
    typedef std::vector<pcl::PointIndices> Indices;
    typedef std::shared_ptr<Indices const> IndicesPtr;

public:
    LabelClusteredPointCloud();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input* input_;
    Input* in_indices_;
    Output* output_;

    IndicesPtr cluster_indices;
};
}  // namespace csapex
#endif  // LABEL_POINTCLOUD_H
