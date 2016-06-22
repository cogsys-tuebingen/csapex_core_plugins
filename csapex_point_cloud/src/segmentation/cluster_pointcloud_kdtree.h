#ifndef CLUSTERPOINTCLOUDKDTREE_H
#define CLUSTERPOINTCLOUDKDTREE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ClusterPointcloudKDTree : public csapex::Node
{
public:
    struct ClusterParams {
        std::array<int, 2>    cluster_sizes;
        std::array<double, 3> bin_sizes;
        std::array<double, 3> cluster_max_std_devs;
        std::array<double, 4> cluster_distance_and_weights;
    };

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
    ClusterParams cluster_params_;

};
}
#endif // CLUSTERPOINTCLOUDKDTREE_H
