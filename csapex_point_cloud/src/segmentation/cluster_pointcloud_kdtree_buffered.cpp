#include "cluster_pointcloud_kdtree_buffered.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <kdtree/buffered_kdtree.hpp>
#include <kdtree/kdtree_cluster_mask.hpp>
#include <kdtree/buffered_kdtree_clustering.hpp>

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


CSAPEX_REGISTER_CLASS(csapex::ClusterPointcloudKDTreeBuffered, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace std;


ClusterPointcloudKDTreeBuffered::ClusterPointcloudKDTreeBuffered()
{
}

void ClusterPointcloudKDTreeBuffered::setupParameters(Parameterizable &parameters)
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
    parameters.addParameter(param::ParameterFactory::declareRange("cloud/max_width", 1, 10000, 640, 1),
                            std::bind(&ClusterPointcloudKDTreeBuffered::updateTree, this, 0));
    parameters.addParameter(param::ParameterFactory::declareRange("cloud/max_height", 1, 10000, 480, 1),
                            std::bind(&ClusterPointcloudKDTreeBuffered::updateTree, this, 0));
}

void ClusterPointcloudKDTreeBuffered::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<ClusterPointcloudKDTreeBuffered>(this, msg), msg->value);
}

void ClusterPointcloudKDTreeBuffered::setup(NodeModifier& node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_ = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
    out_debug_ = node_modifier.addOutput<std::string>("Debug Info");
}

void ClusterPointcloudKDTreeBuffered::updateTree(int size)
{
    if (size == 0)
    {
        const int width = readParameter<int>("cloud/max_width");
        const int height = readParameter<int>("cloud/max_height");
        size = width * height;
    }

    kdtree_ = std::make_shared<kdtree::buffered::KDTree<buffered_tree::BufferedKDTreeNode>>(2 * size + 1);
}

namespace buffered_tree {

template<typename PointT>
void cluster(const TreeType::Ptr& tree,
             const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
             const pcl::PointIndicesPtr &cloud_indeces,
             const PCLKDTreeNodeIndex<PointT> &index,
             std::map<int, std::vector<std::size_t>> &indices)
{
    if(cloud_indeces) {
        for(int i : cloud_indeces->indices) {
            const PointT& point = cloud->at(i);

            if (index.isValid(point))
                tree->insertNode(index.get(point, i));
        }
    } else {
        for(std::size_t i = 0 ; i < cloud->size() ; ++i) {
            const PointT& point = cloud->at(i);

            if (index.isValid(point))
                tree->insertNode(index.get(point, i));
        }
    }

    kdtree::buffered::KDTreeClustering<BufferedKDTreeNode> clustering(tree);
    clustering.cluster();

    std::vector<BufferedKDTreeNode*> leaves;
    tree->getLeaves(leaves);
    for (BufferedKDTreeNode* leaf : leaves) {
        int c = leaf->cluster;
        if (c != -1) {
            indices[c].insert(indices[c].end(),
                              leaf->wrapped.indices.begin(),
                              leaf->wrapped.indices.end());
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
void cluster<pcl::PointNormal>(const typename TreeType::Ptr& tree,
                               const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
                               const pcl::PointIndicesPtr &cloud_indeces,
                               const PCLKDTreeNodeIndex<pcl::PointNormal> &index,
                               std::map<int, std::vector<std::size_t>> &indices)
{
    throw std::runtime_error("pcl::PointNormal not supported!");
}
}

template <class PointT>
void ClusterPointcloudKDTreeBuffered::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    pcl::PointIndicesPtr indices;
    if(msg::isConnected(in_indices_)) {
        auto indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
        indices = indices_msg->value;
    }

    //if (!kdtree_)
        updateTree((indices) ? indices->indices.size() : cloud->size());

    std::map<int, std::vector<std::size_t>> cluster_indices;
    buffered_tree::PCLKDTreeNodeIndex<PointT> index(bin_size_x_,
                                                    bin_size_y_,
                                                    bin_size_z_);

    //kdtree_->resetNodes();

    buffered_tree::cluster<PointT>(kdtree_,
                                   cloud,
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
