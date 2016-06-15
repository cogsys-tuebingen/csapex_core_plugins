#ifndef CLUSTERPOINTCLOUDKDTREE_H
#define CLUSTERPOINTCLOUDKDTREE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <kdtree/buffered_kdtree.hpp>
#include <kdtree/buffered_kdtree_node.hpp>

namespace buffered_tree
{
struct NodeImpl
{
    typedef Eigen::Vector3f MeanType;

    std::vector<std::size_t> indices;
    MeanType    mean = MeanType::Zero();
    std::size_t mean_count = 1;

    inline void overwrite(const NodeImpl& other)
    {
        mean = (mean_count * mean + other.mean_count * other.mean) / (mean_count + other.mean_count);
        mean_count += other.mean_count;
        indices.insert(indices.end(),
                       other.indices.begin(),
                       other.indices.end());
    }
};

typedef kdtree::buffered::KDTreeNode<int, 3, NodeImpl> BufferedKDTreeNode;


template<typename PointT>
struct PCLKDTreeNodeIndex {
    double size_x;
    double size_y;
    double size_z;

    PCLKDTreeNodeIndex(const double size_x,
                       const double size_y,
                       const double size_z) :
        size_x(size_x),
        size_y(size_y),
        size_z(size_z)
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
        node.wrapped.mean_count = 1;
        node.wrapped.mean[0] = point.x;
        node.wrapped.mean[1] = point.y;
        node.wrapped.mean[2] = point.z;
        node.index[0] = floor(point.x / size_x);
        node.index[1] = floor(point.y / size_y);
        node.index[2] = floor(point.z / size_z);

        return node;
    }
};

typedef kdtree::buffered::KDTree<BufferedKDTreeNode> TreeType;
}

namespace csapex {

class ClusterPointcloudKDTreeBuffered : public csapex::Node
{
public:
    ClusterPointcloudKDTreeBuffered();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    void updateTree();

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
    double cluster_distance_;

    typename buffered_tree::TreeType::Ptr kdtree_;

};
}
#endif // CLUSTERPOINTCLOUDKDTREE_H
