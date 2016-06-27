#ifndef CLUSTERPOINTCLOUDKDTREE_H
#define CLUSTERPOINTCLOUDKDTREE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <kdtree/buffered_kdtree.hpp>
#include <kdtree/buffered_kdtree_node.hpp>
#include "../math/distribution.hpp"

namespace buffered_tree
{
struct NodeImpl
{
    typedef Eigen::Vector3d MeanType;

    std::vector<std::size_t> indices;
    math::Distribution<3>    distribution;

    inline void overwrite(const NodeImpl& other)
    {
        distribution += other.distribution;
        indices.insert(indices.end(),
                       other.indices.begin(),
                       other.indices.end());
    }
};

typedef kdtree::buffered::KDTreeNode<int, 3, NodeImpl> BufferedKDTreeNode;


template<typename PointT>
struct PCLKDTreeNodeIndex {
    typedef std::array<double, 3> Size;

    Size size;

    PCLKDTreeNodeIndex(const Size& size) :
        size(size)
    {
    }

    inline bool isValid(const PointT& point) const
    {
        return !(std::isnan(point.x)
                 || std::isnan(point.y)
                 || std::isnan(point.z));
    }

    inline BufferedKDTreeNode get(const PointT &point,
                                  const std::size_t index) const
    {
        BufferedKDTreeNode node;
        node.wrapped.indices.emplace_back(index);
        node.wrapped.distribution.add(Eigen::Vector3d(point.x, point.y, point.z));
        node.index[0] = floor(point.x / size[0]);
        node.index[1] = floor(point.y / size[1]);
        node.index[2] = floor(point.z / size[2]);

        return node;
    }
};

typedef kdtree::buffered::KDTree<BufferedKDTreeNode> TreeType;
}

namespace csapex {

class ClusterPointcloudKDTreeBuffered : public csapex::Node
{
public:
    struct ClusterParams
    {
        enum CovarianceThresholdType {DEFAULT, PCA2D, PCA3D};

        std::array<int, 2>                       cluster_sizes;
        std::array<double, 3>                    bin_sizes;
        std::array<std::pair<double, double>, 3> cluster_std_devs;
        std::array<double, 4>                    cluster_distance_and_weights;
        CovarianceThresholdType                  cluster_cov_thresh_type;

        bool (*validateCovariance)(math::Distribution<3> &distribution,
                                   std::array<std::pair<double, double>, 3> &intervals);
    };

    ClusterPointcloudKDTreeBuffered();

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
    typename buffered_tree::TreeType::Ptr kdtree_;

};
}
#endif // CLUSTERPOINTCLOUDKDTREE_H
