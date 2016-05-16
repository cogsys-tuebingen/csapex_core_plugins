#ifndef CLUSTERPOINTCLOUDKDTREE_H
#define CLUSTERPOINTCLOUDKDTREE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ClusterPointcloudKDTree : public csapex::Node
{
public:
    ClusterPointcloudKDTree();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* in_cloud_;
    Input* in_indices_;
    Output* out_;
    Output* out_debug_;

    double bin_size_x_;
    double bin_size_y_;
    double bin_size_z_;
    int    cluster_min_size_;
    int    cluster_max_size_;

};
}
#endif // CLUSTERPOINTCLOUDKDTREE_H
