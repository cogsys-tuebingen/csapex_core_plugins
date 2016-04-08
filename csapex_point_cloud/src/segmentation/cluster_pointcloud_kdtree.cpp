#include "cluster_pointcloud_kdtree.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <kdtree/kdtree.hpp>
#include <kdtree/kdtree_clustering.hpp>

/// SYSTEM
#include <boost/mpl/for_each.hpp>
#include <tf/tf.h>

/// PCL
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#pragma clang diagnostic ignored "-Wsign-compare"
#endif //__clang__
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#if __clang__
#pragma clang diagnostic pop
#endif //__clang__


CSAPEX_REGISTER_CLASS(csapex::ClusterPointcloudKDTree, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace std;


ClusterPointcloudKDTree::ClusterPointcloudKDTree()
{
}

void ClusterPointcloudKDTree::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("bin/size_x", 0.01, 1.0, 0.1, 0.01),
                            bin_size_x_);
    parameters.addParameter(param::ParameterFactory::declareRange("bin/size_y", 0.01, 1.0, 0.1, 0.01),
                            bin_size_y_);
    parameters.addParameter(param::ParameterFactory::declareRange("bin/size_z", 0.01, 1.0, 0.1, 0.01),
                            bin_size_z_);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/min_size", 1, 1000000, 0, 1),
                            cluster_min_size_);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/max_size", 1, 1000000, 1000000, 1),
                            cluster_max_size_);
}

void ClusterPointcloudKDTree::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<ClusterPointcloudKDTree>(this, msg), msg->value);
}

void ClusterPointcloudKDTree::setup(NodeModifier& node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_ = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
    out_debug_ = node_modifier.addOutput<std::string>("Debug Info");
}

namespace impl {
struct PCLKDTreeNode : public kdtree::KDTreeNode<int,3>
{
    std::vector<size_t> indices;
    PCLKDTreeNode() :
        KDTreeNode()
    {
    }

    inline Ptr clone() const
    {
        PCLKDTreeNode *node = new PCLKDTreeNode(*this);
        node->left = left;
        node->right = right;
        return Ptr(node);
    }

    inline Ptr copy() const
    {
        PCLKDTreeNode *node = new PCLKDTreeNode(*this);
        return Ptr(node);
    }

    inline void overwrite(const Ptr &other) override
    {
        PCLKDTreeNode *other_ptr = (PCLKDTreeNode*) other.get();
        indices.insert(indices.end(),
                       other_ptr->indices.begin(),
                       other_ptr->indices.end());
    }

    inline void split(const Ptr &other) override
    {
        KDTreeNodeType::split(other);
        indices.clear();
    }
};

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

    void get(const PointT &point,
             const std::size_t index,
             PCLKDTreeNode::Ptr &node) const
    {
        if(std::isnan(point.x) ||
                std::isnan(point.y) ||
                std::isnan(point.z))
            return;

        PCLKDTreeNode *n = new PCLKDTreeNode;
        n->indices.emplace_back(index);
        n->index[0] = floor(point.x / size_x);
        n->index[1] = floor(point.y / size_y);
        n->index[2] = floor(point.z / size_z);

        node.reset(n);
    }
};

template<typename PointT>
void cluster(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
             const pcl::PointIndicesPtr &cloud_indeces,
             const PCLKDTreeNodeIndex<PointT> &index,
             std::map<int, std::vector<std::size_t>> &indices)
{
    kdtree::KDTree<int, 3>::Ptr tree(new kdtree::KDTree<int, 3>);
    if(cloud_indeces) {
        for(int i : cloud_indeces->indices) {
            PCLKDTreeNode::Ptr node;

            index.get(cloud->at(i), i, node);
            if(node)
                tree->insertNode(node);
        }
    } else {
        for(std::size_t i = 0 ; i < cloud->size() ; ++i) {
            PCLKDTreeNode::Ptr node;
            index.get(cloud->at(i), i, node);
            if(node)
                tree->insertNode(node);
        }
    }

    kdtree::KDTreeClustering<int,3> clustering(tree);
    clustering.cluster();

    std::vector<PCLKDTreeNode::Ptr> leaves;
    tree->getLeaves(leaves);
    for(PCLKDTreeNode::Ptr &leaf : leaves) {
        PCLKDTreeNode* l = (PCLKDTreeNode*) leaf.get();
        int c = l->cluster;
        if(c != -1) {
            indices[c].insert(indices[c].end(),
                              l->indices.begin(),
                              l->indices.end());
        }
    }

}

template<>
struct PCLKDTreeNodeIndex<pcl::PointNormal>
{
    PCLKDTreeNodeIndex(const double size_x,
                       const double size_y,
                       const double size_z)
    {
    }

};

template<>
void cluster<pcl::PointNormal>(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
                               const pcl::PointIndicesPtr &cloud_indeces,
                               const PCLKDTreeNodeIndex<pcl::PointNormal> &index,
                               std::map<int, std::vector<std::size_t>> &indices)
{
    throw std::runtime_error("pcl::PointNormal not supported!");
}
}

template <class PointT>
void ClusterPointcloudKDTree::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    pcl::PointIndicesPtr indices;
    if(msg::isConnected(in_indices_)) {
        auto indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
        indices = indices_msg->value;
    }

    std::map<int, std::vector<std::size_t>> cluster_indices;
    impl::PCLKDTreeNodeIndex<PointT> index(bin_size_x_,
                                           bin_size_y_,
                                           bin_size_z_);
    impl::cluster<PointT>(cloud,
                          indices,
                          index,
                          cluster_indices);

    std::shared_ptr<std::vector<pcl::PointIndices> >
            out_cluster_indices(new std::vector<pcl::PointIndices>);
    for(auto &cluster : cluster_indices) {
        if(cluster.second.size() < (std::size_t) cluster_min_size_)
            continue;
        if(cluster.second.size() > (std::size_t) cluster_max_size_)
            continue;

        out_cluster_indices->emplace_back(pcl::PointIndices());
        pcl::PointIndices &entry = out_cluster_indices->back();
        for(std::size_t index : cluster.second) {
            entry.indices.emplace_back(index);
        }
    }

    std::stringstream stringstream;
    stringstream << "Found clusters: " << out_cluster_indices->size();
    std::string text_msg = stringstream.str();
    msg::publish(out_debug_, text_msg);
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, out_cluster_indices);
}
