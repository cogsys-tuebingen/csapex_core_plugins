#ifndef CLUSTERPOINTCLOUDKDTREE_H
#define CLUSTERPOINTCLOUDKDTREE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <kdtree/kdtree.hpp>
#include "../math/distribution.hpp"

namespace detail_filtered
{
struct NodeIndex
{
    using                           BinType     = std::array<double, 3>;
    using                           Type        = std::array<int, 3>;
    using                           PivotType   = double;
    static constexpr std::size_t    Dimension   = 3;

    const BinType bin_sizes;

    NodeIndex(const BinType& bin_sizes) : bin_sizes(bin_sizes) {}

    template<typename PointT>
    inline constexpr bool is_valid(const PointT& point) const
    {
        return !(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z));
    }

    template<typename PointT>
    inline constexpr Type create(const PointT& point) const
    {
        return { static_cast<int>(std::floor(point.x / bin_sizes[0])),
                 static_cast<int>(std::floor(point.y / bin_sizes[1])),
                 static_cast<int>(std::floor(point.z / bin_sizes[2]))};
    }
};

struct NodeData : public kdtree::KDTreeNodeClusteringSupport
{
    std::vector<std::size_t> indices;
    math::Distribution<3>    distribution;

    NodeData() = default;

    template<typename PointT>
    inline NodeData(const PointT& point, std::size_t index)
    {
        indices.emplace_back(index);
        distribution.add({point.x, point.y, point.z});
    }

    inline void merge(const NodeData& other)
    {
        distribution += other.distribution;
        indices.insert(indices.end(),
                       other.indices.begin(),
                       other.indices.end());
    }
};

using KDTree = kdtree::buffered::KDTree<NodeIndex, NodeData>;
using KDTreePtr = std::shared_ptr<KDTree>;
}

namespace csapex
{

class ClusterPointcloudKDTreeFiltered : public csapex::Node
{
public:
    struct ClusterParams
    {
        enum CovarianceThresholdType {DEFAULT, PCA2D, PCA3D};

        std::array<double, 3>                    bin_sizes;
        std::array<int, 2>                       cluster_sizes;
        std::array<std::pair<double, double>, 3> cluster_std_devs;
        std::array<double, 4>                    cluster_distance_and_weights;
        CovarianceThresholdType                  cluster_cov_thresh_type;

        bool (*validateCovariance)(math::Distribution<3> &distribution,
                                   std::array<std::pair<double, double>, 3> &intervals);
    };

    ClusterPointcloudKDTreeFiltered();

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

    std::size_t last_size_;
    detail_filtered::KDTreePtr kdtree_;

};
}
#endif // CLUSTERPOINTCLOUDKDTREE_H
