#include "cluster_kdtree_filtered.h"

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
#include "cluster_utils.hpp"

CSAPEX_REGISTER_CLASS(csapex::ClusterPointcloudKDTreeFiltered, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


ClusterPointcloudKDTreeFiltered::ClusterPointcloudKDTreeFiltered() :
    last_size_(0)
{
}

void ClusterPointcloudKDTreeFiltered::setupParameters(Parameterizable &parameters)
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

void ClusterPointcloudKDTreeFiltered::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor(PointCloudMessage::Dispatch<ClusterPointcloudKDTreeFiltered>(this, msg), msg->value);
}

void ClusterPointcloudKDTreeFiltered::setup(NodeModifier& node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_ = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Clusters");
    out_rejected_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Rejected Clusters");
    out_debug_ = node_modifier.addOutput<std::string>("Debug Info");
}

namespace detail_filtered
{

typedef ClusterPointcloudKDTreeFiltered::ClusterParams ClusterParams;
typedef Validator<ClusterParams>                       ValidatorType;

template<typename PointT>
void cluster(const KDTreePtr&                                       tree,
             const typename pcl::PointCloud<PointT>::ConstPtr&      cloud,
             const pcl::PointIndices::ConstPtr&                     cloud_indices,
             const ClusterPointcloudKDTreeFiltered::ClusterParams&  params,
             ClusterPointcloudKDTreeFiltered*                       self,
             std::vector<pcl::PointIndices>&                        indicies,
             std::shared_ptr<std::vector<pcl::PointIndices>>&       rejected)
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
        math::Distribution<3> buffer_distribution;

        using MeanType = math::Distribution<3>::PointType;
        const double max_distance = params.cluster_distance_and_weights[0];
        const double w_0 = params.cluster_distance_and_weights[1];
        const double w_1 = params.cluster_distance_and_weights[2];
        const double w_2 = params.cluster_distance_and_weights[3];

        ValidatorType validator(params, buffer_indices, buffer_distribution);

        kdtree::KDTreeClustering<KDTree> clustering(*tree);

        clustering.set_cluster_init([&](const NodeData& data)
        {
            ValidatorType::Result validation = validator.validate();
            if (validation == ValidatorType::Result::ACCEPTED)
                indicies.emplace_back(std::move(buffer_indices));
            else
            {
                if (rejected && validation != ValidatorType::Result::TOO_SMALL)
                    rejected->emplace_back(std::move(buffer_indices));
                else
                    buffer_indices.indices.clear();
            }

            buffer_distribution.reset();

            return true;
        });

        clustering.set_cluster_extend([&](const NodeData& node_data, const NodeData& neighbour_data)
        {
            if (max_distance != 0)
            {
                MeanType diff = node_data.distribution.getMean() - neighbour_data.distribution.getMean();
                diff(0) *= w_0;
                diff(1) *= w_1;
                diff(2) *= w_2;
                auto dist = diff.dot(diff);
                if (dist > max_distance)
                    return false;
            }

            buffer_distribution += neighbour_data.distribution;
            buffer_indices.indices.insert(buffer_indices.indices.end(), neighbour_data.indices.begin(), neighbour_data.indices.end());

            return true;
        });

        clustering.cluster();

        ValidatorType::Result validation = validator.validate();
        if (validation == ValidatorType::Result::ACCEPTED)
            indicies.emplace_back(std::move(buffer_indices));
        else if (rejected && validation != ValidatorType::Result::TOO_SMALL)
            rejected->emplace_back(std::move(buffer_indices));
    }

}
}

template <class PointT>
void ClusterPointcloudKDTreeFiltered::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
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

        if (!kdtree_  || size > last_size_)
        {
            kdtree_.reset(new detail_filtered::KDTree(2 * size + 1));
            last_size_ = size;
        }
        else
            kdtree_->clear();

    }

    std::shared_ptr<std::vector<pcl::PointIndices>> out_cluster_indices = std::make_shared<std::vector<pcl::PointIndices>>();
    std::shared_ptr<std::vector<pcl::PointIndices>> out_rejected_indices;
    if (msg::isConnected(out_rejected_))
            out_rejected_indices = std::make_shared<std::vector<pcl::PointIndices>>();

    detail_filtered::cluster<PointT>(kdtree_,
                                     cloud,
                                     indices,
                                     cluster_params_,
                                     this,
                                     *out_cluster_indices,
                                     out_rejected_indices);


    std::stringstream stringstream;
    stringstream << "Found clusters: " << out_cluster_indices->size();
    std::string text_msg = stringstream.str();
    msg::publish(out_debug_, text_msg);
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, out_cluster_indices);
    if (msg::isConnected(out_rejected_))
        msg::publish<GenericVectorMessage, pcl::PointIndices >(out_rejected_, out_rejected_indices);
}
