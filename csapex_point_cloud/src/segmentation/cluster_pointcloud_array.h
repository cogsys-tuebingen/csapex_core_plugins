#ifndef CLUSTERPOINTCLOUDARRAY_H
#define CLUSTERPOINTCLOUDARRAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ClusterPointCloudArray : public csapex::Node
{
public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    struct ClusterParams {
        std::array<double, 3> bin_sizes;
        std::array<int, 2>    cluster_sizes;
    };

    Input*        in_cloud_;
    Input*        in_indices_;
    Output*       out_;

    ClusterParams cluster_params_;

};
}

#endif // CLUSTERPOINTCLOUDARRAY_H
