#ifndef CLUSTERPOINTCLOUDARRAY_H
#define CLUSTERPOINTCLOUDARRAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include "../math/distribution.hpp"

namespace csapex {
class ClusterPointCloudArrayFiltered : public csapex::Node
{
public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    struct ClusterParams {
        enum CovarianceThresholdType {DEFAULT, PCA2D, PCA3D};

        std::array<double, 3>                    bin_sizes;
        std::array<int, 2>                       cluster_sizes;
        std::array<std::pair<double, double>, 3> cluster_std_devs;
        std::array<double, 4>                    cluster_distance_and_weights;
        CovarianceThresholdType                  cluster_cov_thresh_type;

        bool (*validateCovariance)(math::Distribution<3> &distribution,
                                   std::array<std::pair<double, double>, 3> &intervals);
    };

private:
    Input*        in_cloud_;
    Input*        in_indices_;
    Output*       out_;
    Output*       out_rejected_;

    ClusterParams cluster_params_;

};
}

#endif // CLUSTERPOINTCLOUDARRAY_H
