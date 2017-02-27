#include "cluster.hpp"
#include "data/cluster_data.hpp"
#include "storage/storage_ops.hpp"
#include "storage/cluster_op.hpp"

#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/utility/register_apex_plugin.h>

#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/indeces_message.h>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backends.hpp>

using namespace csapex;
using namespace csapex::connection_types;
namespace cis = cslibs_indexed_storage;

void ClusterPointCloud::setupParameters(Parameterizable& parameters)
{
    static const std::map<std::string, int> backend_types{
            { "page", static_cast<int>(BackendType::PAGED) },
            { "k-d tree", static_cast<int>(BackendType::KDTREE) },
            { "array", static_cast<int>(BackendType::ARRAY) },
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("backend",
                                                                         backend_types,
                                                                         static_cast<int>(BackendType::PAGED)),
                            reinterpret_cast<int&>(backend_));

    parameters.addParameter(param::ParameterFactory::declareRange("voxel/size/x", 0.01, 10.0, 0.1, 0.01),
                            voxel_size_[0]);
    parameters.addParameter(param::ParameterFactory::declareRange("voxel/size/y", 0.01, 10.0, 0.1, 0.01),
                            voxel_size_[1]);
    parameters.addParameter(param::ParameterFactory::declareRange("voxel/size/z", 0.01, 10.0, 0.1, 0.01),
                            voxel_size_[2]);

    parameters.addParameter(param::ParameterFactory::declareInterval("filter/point_count", 1, 10000000, 1, 10000000, 1),
                           cluster_point_count_);

    param::ParameterPtr param_distribution = param::ParameterFactory::declareBool("filter/distribution", false);
    std::function<bool()> enable_distribution = [param_distribution]() -> bool  { return param_distribution->as<bool>(); };
    static const std::map<std::string, int> distribution_types{
            { "DEFAULT", static_cast<int>(ClusterDistributionAnalysisType::DEFAULT) },
            { "PCA2D", static_cast<int>(ClusterDistributionAnalysisType::PCA2D) },
            { "PCA3D", static_cast<int>(ClusterDistributionAnalysisType::PCA3D) },
    };

    parameters.addParameter(param_distribution,
                            distribution_enabled_);
    parameters.addConditionalParameter(param::ParameterFactory::declareParameterSet("filter/distribution/type",
                                                                                    distribution_types,
                                                                                    static_cast<int>(ClusterDistributionAnalysisType::DEFAULT)),
                                       enable_distribution,
                                       reinterpret_cast<int&>(distribution_type_));
    parameters.addConditionalParameter(param::ParameterFactory::declareInterval("filter/distribution/std_dev/x", 0.0, 10.0, 0.0, 10.0, 0.01),
                                       enable_distribution,
                                       distribution_std_dev_[0]);
    parameters.addConditionalParameter(param::ParameterFactory::declareInterval("filter/distribution/std_dev/y", 0.0, 10.0, 0.0, 10.0, 0.01),
                                       enable_distribution,
                                       distribution_std_dev_[1]);
    parameters.addConditionalParameter(param::ParameterFactory::declareInterval("filter/distribution/std_dev/z", 0.0, 10.0, 0.0, 10.0, 0.01),
                                       enable_distribution,
                                       distribution_std_dev_[2]);

    param::ParameterPtr param_color = param::ParameterFactory::declareBool("filter/color", false);
    std::function<bool()> enable_color = [param_color]() -> bool { return param_color->as<bool>(); };
    static const std::map<std::string, int> color_types{
            { "CIE76", static_cast<int>(ClusterColorType::CIE76) },
            { "CIE94Grahpics", static_cast<int>(ClusterColorType::CIE94Grahpics) },
            { "CIE94Textiles", static_cast<int>(ClusterColorType::CIE94Textiles) },
    };

    parameters.addParameter(param_color,
                            color_enabled_);
    parameters.addConditionalParameter(param::ParameterFactory::declareParameterSet("filter/color/type",
                                                                                    color_types,
                                                                                    static_cast<int>(ClusterColorType::CIE76)),
                                       enable_color,
                                       reinterpret_cast<int&>(color_type_));
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/color/max_difference", 0.0, 4000.0, 0.0, 0.1),
                                       enable_color,
                                       color_threshold_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/color/weights/l", 0.0, 1.0, 1.0, 0.01),
                                       enable_color,
                                       color_weights_[0]);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/color/weights/a", 0.0, 1.0, 1.0, 0.01),
                                       enable_color,
                                       color_weights_[1]);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/color/weights/b", 0.0, 1.0, 1.0, 0.01),
                                       enable_color,
                                       color_weights_[2]);

}

void ClusterPointCloud::setup(NodeModifier& node_modifier)
{
    in_pointcloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_    = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_clusters_accepted_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Clusters (accepted)");
    out_clusters_rejected_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Clusters (rejected)");
}

void ClusterPointCloud::process()
{
    PointCloudMessage::ConstPtr pointcloud_message(msg::getMessage<PointCloudMessage>(in_pointcloud_));
    boost::apply_visitor(PointCloudMessage::Dispatch<ClusterPointCloud>(this, pointcloud_message),
                         pointcloud_message->value);
}

template<typename PointT>
void ClusterPointCloud::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    selectData<PointT>(cloud);
}

template<typename PointT>
void ClusterPointCloud::selectData(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if (distribution_enabled_ && color_enabled_)
    {
        using Data = ClusterData<ClusterFeatureDistribution, ClusterFeatureColor>;
        selectStorage<Data, PointT>(cloud);
    }
    else if (distribution_enabled_)
    {
        using Data = ClusterData<ClusterFeatureDistribution>;
        selectStorage<Data, PointT>(cloud);
    }
    else if (color_enabled_)
    {
        using Data = ClusterData<ClusterFeatureColor>;
        selectStorage<Data, PointT>(cloud);
    }
    else
    {
        using Data = ClusterData<>;
        selectStorage<Data, PointT>(cloud);
    }
}


template<typename DataType, typename PointT>
void ClusterPointCloud::selectStorage(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    switch (backend_)
    {
        default:
        case BackendType::PAGED:
            return clusterCloud<cis::AutoIndexStorage<DataType, cis::backend::simple::UnorderedComponentMap>, PointT>(cloud);
        case BackendType::KDTREE:
            return clusterCloud<cis::AutoIndexStorage<DataType, cis::backend::kdtree::KDTreeBuffered>, PointT>(cloud);
    }
}

template<typename Storage, typename PointT>
void ClusterPointCloud::clusterCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if (cloud->empty())
        return;

    pcl::PointIndices::ConstPtr input_indices;
    if (msg::isConnected(in_indices_))
    {
        auto indices_message = msg::getMessage<PointIndecesMessage>(in_indices_);
        input_indices = indices_message->value;
    }

    auto clusters_accepted_message_ = std::make_shared<std::vector<pcl::PointIndices>>();
    auto clusters_rejected_message_ = std::make_shared<std::vector<pcl::PointIndices>>();

    using DataType = typename Storage::data_t;
    using StorageOps = ClusterStorageOps<true>;
    using Clusterer = ClusterOp<Storage, DistributionValidator<DataType>, ColorValidator<DataType>>;

    Storage storage;
    {
        NAMED_INTERLUDE(init);
        ClusterIndex indexer(voxel_size_[0], voxel_size_[1], voxel_size_[2]);
        StorageOps::init(*cloud, input_indices, indexer, storage);
    }

    DistributionValidator<DataType> distribution_validator(distribution_type_,
                                                           distribution_std_dev_);
    ColorValidator<DataType> color_validator(color_type_,
                                             color_weights_,
                                             color_threshold_);

    Clusterer cluster_op(storage,
                         distribution_validator,
                         color_validator);
    {
        NAMED_INTERLUDE(cluster);
        cis::operations::clustering::Clustering<Storage> clustering(storage);
        clustering.cluster(cluster_op);
    }

    {
        NAMED_INTERLUDE(extract);
        StorageOps::extract(storage,
                            cluster_op,
                            *clusters_accepted_message_,
                            *clusters_rejected_message_);
    }

    {
        NAMED_INTERLUDE(count_filter);
        const auto filter = [this](std::vector<pcl::PointIndices>& indices)
        {
            indices.erase(std::remove_if(indices.begin(), indices.end(),
                                         [this](const pcl::PointIndices& list)
                                         {
                                             const int size = static_cast<int>(list.indices.size());
                                             return !(size >= cluster_point_count_.first
                                                      && size <= cluster_point_count_.second);
                                         }),
                          indices.end());
        };

        filter(*clusters_accepted_message_);
        filter(*clusters_rejected_message_);
    }

    msg::publish<GenericVectorMessage, pcl::PointIndices>(out_clusters_accepted_, clusters_accepted_message_);
    msg::publish<GenericVectorMessage, pcl::PointIndices>(out_clusters_rejected_, clusters_rejected_message_);
}

CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloud, csapex::Node)
