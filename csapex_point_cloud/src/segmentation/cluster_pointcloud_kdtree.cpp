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
#include <kdtree/kdtree_cluster_mask.hpp>

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
    parameters.addParameter(param::ParameterFactory::declareRange("bin/size_x", 0.01, 8.0, 0.1, 0.01),
                            cluster_params_.bin_sizes[0]);
    parameters.addParameter(param::ParameterFactory::declareRange("bin/size_y", 0.01, 8.0, 0.1, 0.01),
                            cluster_params_.bin_sizes[1]);
    parameters.addParameter(param::ParameterFactory::declareRange("bin/size_z", 0.01, 8.0, 0.1, 0.01),
                            cluster_params_.bin_sizes[2]);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/min_size", 1, 1000000, 0, 1),
                            cluster_params_.cluster_sizes[0]);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/max_size", 1, 1000000, 1000000, 1),
                            cluster_params_.cluster_sizes[1]);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/max_distance", 0.00, 3.0, 0.0, 0.01),
                            cluster_params_.cluster_distance_and_weights[0]);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/distance_weights/x", 0.0, 1.0, 1.0, 0.01),
                            cluster_params_.cluster_distance_and_weights[1]);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/distance_weights/y", 0.0, 1.0, 1.0, 0.01),
                            cluster_params_.cluster_distance_and_weights[2]);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/distance_weights/z", 0.0, 1.0, 1.0, 0.01),
                            cluster_params_.cluster_distance_and_weights[3]);
    parameters.addParameter(param::ParameterFactory::declareInterval("cluster/std_dev/x", 0.0, 3.0, 0.0, 0.0, 0.01),
                            cluster_params_.cluster_std_devs[0]);
    parameters.addParameter(param::ParameterFactory::declareInterval("cluster/std_dev/y", 0.0, 3.0, 0.0, 0.0, 0.01),
                            cluster_params_.cluster_std_devs[1]);
    parameters.addParameter(param::ParameterFactory::declareInterval("cluster/std_dev/z", 0.0, 3.0, 0.0, 0.0, 0.01),
                            cluster_params_.cluster_std_devs[2]);

    std::map<std::string, int> covariance_threshold_types = {{"DEFAULT", ClusterParams::DEFAULT},
                                                             {"PCA2D", ClusterParams::PCA2D},
                                                             {"PCA3D", ClusterParams::PCA3D}};
    parameters.addParameter(param::ParameterFactory::declareParameterSet("cluster/std_dev_thresh_type",
                                                                         covariance_threshold_types,
                                                                         (int) ClusterParams::DEFAULT),
                            reinterpret_cast<int&>(cluster_params_.cluster_cov_thresh_type));

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
    typedef Eigen::Vector3d MeanType;

    std::list<size_t>     indices;
    math::Distribution<3> distribution;

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
        distribution += other_ptr->distribution;
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
    typedef std::array<double, 3> Size;

    Size size;
    PCLKDTreeNodeIndex(const Size size) :
        size(size)
    {
    }

    inline void get(const PointT &point,
                    const std::size_t index,
                    PCLKDTreeNode::Ptr &node) const
    {
        if(std::isnan(point.x) ||
                std::isnan(point.y) ||
                std::isnan(point.z))
            return;

        PCLKDTreeNode *n = new PCLKDTreeNode;
        n->indices.emplace_back(index);
        n->distribution.add(Eigen::Vector3d(point.x, point.y, point.z));
        n->index[0] = floor(point.x / size[0]);
        n->index[1] = floor(point.y / size[1]);
        n->index[2] = floor(point.z / size[2]);

        node.reset(n);
    }
};

/// TODO: calculate mean of kd tree voxel -> insert eucl. distance thresh

template<typename T, int Dim>
struct KDTreeClustering {
    typedef kdtree::KDTree<T, Dim>                 KDTreeType;
    typedef typename KDTreeType::NodeIndex         NodeIndex;
    typedef typename KDTreeType::NodePtr           NodePtr;
    typedef ClusterPointcloudKDTree::ClusterParams ClusterParams;

    typename KDTreeType::Ptr        kdtree;
    kdtree::KDTreeClusterMask<Dim>  cluster_mask;
    std::vector<NodePtr>            queue;
    std::size_t                     cluster_count;
    ClusterParams                   params;

    inline void square(double &value)
    {
        value *= value;
    }

    KDTreeClustering(const typename KDTreeType::Ptr &kdtree,
                     const ClusterPointcloudKDTree::ClusterParams &params) :
        kdtree(kdtree),
        cluster_count(0),
        params(params)
    {
        square(this->params.cluster_distance_and_weights[0]);
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            square(this->params.cluster_std_devs[i].first);
            square(this->params.cluster_std_devs[i].second);
        }
        switch(params.cluster_cov_thresh_type) {
        case ClusterParams::DEFAULT:
            this->params.validateCovariance = KDTreeClustering::validateCovDefault;
            break;
        case ClusterParams::PCA2D:
            this->params.validateCovariance = KDTreeClustering::validateCovPCA2D;
            break;
        case ClusterParams::PCA3D:
            this->params.validateCovariance = KDTreeClustering::validateCovPCA3D;
            break;
        default:
            this->params.validateCovariance = KDTreeClustering::validateCovDefault;
            break;
        }
   }

    int getCluster(const NodeIndex &index) {
        NodePtr node;
        if(!kdtree->find(index, node))
            return -1;
        return node->cluster;
    }

    inline void cluster(std::vector<pcl::PointIndices> &indices)
    {
        queue.reserve(kdtree->leafCount());
        kdtree->getLeaves(queue, true);

        pcl::PointIndices     buffer_indices;
        math::Distribution<3> buffer_distribution;

        for(NodePtr &node : queue) {
            PCLKDTreeNode *n = (PCLKDTreeNode*) node.get();
            /// node already clustered
            if(n->cluster > -1) {
                continue;
            } else {
                std::size_t size = buffer_indices.indices.size();
                if(size > 0) {
                    if(validateSize(size) &&
                            (*(params.validateCovariance))(buffer_distribution, params.cluster_std_devs)) {
                        indices.emplace_back(buffer_indices);
                    }
                }

                buffer_indices.indices.clear();
                buffer_distribution.reset();
            }

            /// spawn a new cluster
            n->cluster = cluster_count;
            ++cluster_count;

            clusterNode(node, buffer_indices, buffer_distribution);
        }

        std::size_t size = buffer_indices.indices.size();
        if(size > 0) {
            if(validateSize(size) &&
                  (*(params.validateCovariance))(buffer_distribution, params.cluster_std_devs)) {
                indices.emplace_back(buffer_indices);
            }
        }
    }

    inline bool validateSize(std::size_t size)
    {
        return size >= params.cluster_sizes[0] &&
               size <= params.cluster_sizes[1];
    }

    inline static bool validateCovDefault(math::Distribution<3> &distribution,
                                          std::array<std::pair<double, double>, 3> &intervals)
    {
        bool valid = true;
        math::Distribution<3>::MatrixType cov;
        distribution.getCovariance(cov);
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            const auto &interval = intervals[i];
            valid &= cov(i,i) >= interval.first;
            valid &= (interval.second == 0.0 || cov(i,i) <= interval.second);
        }
        return valid;
    }

    inline static bool validateCovPCA2D(math::Distribution<3> &distribution,
                                        std::array<std::pair<double, double>, 3> &intervals)
    {
        bool valid = true;
        math::Distribution<3>::MatrixType cov3D;
        distribution.getCovariance(cov3D);

        Eigen::Matrix2d cov2D = cov3D.block<2,2>(0,0);
        Eigen::EigenSolver<Eigen::Matrix2d> solver(cov2D);
        Eigen::Vector2d eigen_values  = solver.eigenvalues().real();

        for(std::size_t i = 0 ; i < 2 ; ++i) {
            const auto &interval = intervals[i];
            valid &= eigen_values[i] >= interval.first;
            valid &= (interval.second == 0.0 || eigen_values[i] <= interval.second);
        }

        return valid;
    }

    inline static bool validateCovPCA3D(math::Distribution<3> &distribution,
                                        std::array<std::pair<double, double>, 3> &intervals)
    {
        bool valid = true;
        math::Distribution<3>::EigenValueSetType eigen_values;
        distribution.getEigenValues(eigen_values, true);
        /// first sort the eigen values by descending so first paramter always corresponds to
        /// the highest value
        std::vector<double> eigen_values_vec(eigen_values.data(), eigen_values.data() + 3);
        std::sort(eigen_values_vec.begin(), eigen_values_vec.end());

        for(std::size_t i = 0 ; i < 3 ; ++i) {
            const auto &interval = intervals[i];
            valid &= eigen_values_vec[i] >= interval.first;
            valid &= (interval.second == 0.0 || eigen_values_vec[i] <= interval.second);
        }
        return valid;
    }

    inline void clusterNode(NodePtr           &node,
                            pcl::PointIndices &indices,
                            math::Distribution<3> &distribution)
    {
        /// check surrounding indeces
        NodeIndex index;
        const double max_distance = params.cluster_distance_and_weights[0];
        PCLKDTreeNode::MeanType diff;
        if(max_distance != 0) {
            const double w_0 = params.cluster_distance_and_weights[1];
            const double w_1 = params.cluster_distance_and_weights[2];
            const double w_2 = params.cluster_distance_and_weights[3];
            for(std::size_t i = 0 ; i < cluster_mask.rows ; ++i) {
                cluster_mask.applyToIndex(node->index, i, index);
                NodePtr neighbour;
                if(!kdtree->find(index, neighbour))
                    continue;
                if(neighbour->cluster > -1)
                    continue;

                PCLKDTreeNode *neighbour_ptr = (PCLKDTreeNode*) neighbour.get();
                PCLKDTreeNode *node_ptr = (PCLKDTreeNode*) node.get();
                diff = node_ptr->distribution.getMean() - neighbour_ptr->distribution.getMean();
                diff(0) *= w_0;
                diff(1) *= w_1;
                diff(2) *= w_2;
                auto dist = diff.dot(diff);
                if (dist > max_distance)
                    continue;

                distribution += neighbour_ptr->distribution;
                neighbour_ptr->cluster = node->cluster;
                indices.indices.insert(indices.indices.end(), neighbour_ptr->indices.begin(), neighbour_ptr->indices.end());

                clusterNode(neighbour, indices, distribution);
            }
        } else {
            for(std::size_t i = 0 ; i < cluster_mask.rows ; ++i) {
                cluster_mask.applyToIndex(node->index, i, index);
                NodePtr neighbour;
                if(!kdtree->find(index, neighbour))
                    continue;
                if(neighbour->cluster > -1)
                    continue;
                PCLKDTreeNode *neighbour_ptr = (PCLKDTreeNode*) neighbour.get();
                distribution += neighbour_ptr->distribution;
                neighbour_ptr->cluster = node->cluster;
                indices.indices.insert(indices.indices.end(), neighbour_ptr->indices.begin(), neighbour_ptr->indices.end());
                clusterNode(neighbour, indices, distribution);
            }
        }
    }
};


/// initiate clustering -----------------------------------------------------------------
template<typename PointT>
void cluster(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
             const pcl::PointIndicesPtr                       &cloud_indeces,
             const ClusterPointcloudKDTree::ClusterParams     &params,
             ClusterPointcloudKDTree                         *self,
             std::vector<pcl::PointIndices>                   &indices)
{

    PCLKDTreeNodeIndex<PointT>  index(params.bin_sizes);
    kdtree::KDTree<int, 3>::Ptr tree(new kdtree::KDTree<int, 3>);

    {
        NAMED_INTERLUDE_INSTANCE(self, build_tree);

        if(cloud_indeces) {
            for(const int i : cloud_indeces->indices) {
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
    }
    {
        NAMED_INTERLUDE_INSTANCE(self, cluster_tree);

        KDTreeClustering<int,3> clustering(tree, params);
        clustering.cluster(indices);
    }
}
/// -------------------------------------------------------------------------------------
/// dead end implementaions -------------------------------------------------------------
template<>
struct PCLKDTreeNodeIndex<pcl::PointNormal>
{
    typedef std::array<double, 3> Size;

    PCLKDTreeNodeIndex(const Size size)
    {
    }

};

template<>
void cluster<pcl::PointNormal>(const typename pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
                               const pcl::PointIndicesPtr                                 &cloud_indeces,
                               const ClusterPointcloudKDTree::ClusterParams               &params,
                               ClusterPointcloudKDTree                                    *self,
                               std::vector<pcl::PointIndices>                             &indices)
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

    std::shared_ptr<std::vector<pcl::PointIndices> >
            out_cluster_indices(new std::vector<pcl::PointIndices>);

    /// todo : minimum covariance?
    /// todo : allow main components for evaluation (rotation of cluster does not matter then)
    impl::cluster<PointT>(cloud,
                          indices,
                          cluster_params_,
                          this,
                          *out_cluster_indices);

    std::stringstream stringstream;
    stringstream << "Found clusters: " << out_cluster_indices->size();
    std::string text_msg = stringstream.str();
    msg::publish(out_debug_, text_msg);
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, out_cluster_indices);
}
