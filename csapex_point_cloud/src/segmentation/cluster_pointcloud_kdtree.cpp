#include "cluster_pointcloud_kdtree.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>


CSAPEX_REGISTER_CLASS(csapex::ClusterPointcloudKDTree, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


ClusterPointcloudKDTree::ClusterPointcloudKDTree() :
    last_size_(0)
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

namespace detail {

template<typename PointT>
void cluster(const KDTreePtr&                                       tree,
             const typename pcl::PointCloud<PointT>::ConstPtr&      cloud,
             const pcl::PointIndices::ConstPtr&                     cloud_indices,
             const ClusterPointcloudKDTree::ClusterParams&          params,
             ClusterPointcloudKDTree*                               self,
             std::vector<pcl::PointIndices>&                        indicies)
{
    {
        NAMED_INTERLUDE_INSTANCE(self, build_tree);

        NodeIndex index(params.bin_sizes);

        if (cloud_indices)
        {
            for (int i : cloud_indices->indices)
            {
                const PointT& point = cloud->at(i);
                if (index.is_valid(point))
                    tree->insert_bulk(index.create(point), NodeData(point, i));
            }
        }
        else
        {
            for (std::size_t i = 0 ; i < cloud->size() ; ++i)
            {
                const PointT& point = cloud->at(i);
                if (index.is_valid(point))
                    tree->insert_bulk(index.create(point), NodeData(point, i));
            }
        }
        tree->load_bulk();
    }
    {
        NAMED_INTERLUDE_INSTANCE(self, cluster_tree);


        pcl::PointIndices     buffer_indices;

        kdtree::KDTreeClustering<KDTree> clustering(*tree);

        clustering.set_cluster_init([&](const NodeData& data)
        {
            if (buffer_indices.indices.size() >= params.cluster_sizes[0] &&
                buffer_indices.indices.size() <= params.cluster_sizes[1])
                indicies.emplace_back(std::move(buffer_indices));
            else
                buffer_indices.indices.clear();

            return true;
        });

        clustering.set_cluster_extend([&](const NodeData& node_data, const NodeData& neighbour_data)
        {
            buffer_indices.indices.insert(buffer_indices.indices.end(), neighbour_data.indices.begin(), neighbour_data.indices.end());

            return true;
        });

        clustering.cluster();

        if (buffer_indices.indices.size() >= params.cluster_sizes[0] &&
            buffer_indices.indices.size() <= params.cluster_sizes[1])
            indicies.emplace_back(std::move(buffer_indices));
    }

}
}

template <class PointT>
void ClusterPointcloudKDTree::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if (cloud->empty())
        return;

    std::size_t size = cloud->size();
    pcl::PointIndicesPtr indices;
    if(msg::isConnected(in_indices_))
    {
        auto indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
        indices = indices_msg->value;
    }


    {
        NAMED_INTERLUDE(init_tree);

        if (!kdtree_ || size > last_size_)
        {
            kdtree_.reset(new detail::KDTree(2 * size + 1));
            last_size_ = size;
        }
        else
            kdtree_->clear();

    }

    std::shared_ptr<std::vector<pcl::PointIndices>> out_cluster_indices(new std::vector<pcl::PointIndices>);

    detail::cluster<PointT>(kdtree_,
                            cloud,
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
