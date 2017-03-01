#include "cluster.hpp"
#include "data/voxel_data.hpp"
#include "storage/storage_ops.hpp"
#include "storage/cluster_op.hpp"

#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/view/utility/color.hpp>

#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/indeces_message.h>
#include <cslibs_indexed_storage/backends.hpp>

#include <boost/make_shared.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex::clustering;
namespace cis = cslibs_indexed_storage;

namespace
{
struct Color
{
    Color(uint8_t r, uint8_t g, uint8_t b) :
            r(r), g(g), b(b)
    {}

    uint8_t r;
    uint8_t g;
    uint8_t b;
};
}

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


    param::ParameterPtr param_voxel_validation = param::ParameterFactory::declareBool("filter/voxel_validation", false);
    std::function<bool()> enable_voxel_validation = [param_voxel_validation]() -> bool { return param_voxel_validation->as<bool>(); };

    parameters.addParameter(param_voxel_validation,
                            voxel_validation_enabled_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/voxel_validation/min_count", 1, 1000, 1, 1),
                                       enable_voxel_validation,
                                       voxel_validation_min_count_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/voxel_validation/scale", 0.0, 1.0, 0.0, 0.001),
                                       enable_voxel_validation,
                                       voxel_validation_scale_);


    param::ParameterPtr param_distribution = param::ParameterFactory::declareBool("filter/distribution", false);
    std::function<bool()> enable_distribution = [param_distribution]() -> bool  { return param_distribution->as<bool>(); };
    static const std::map<std::string, int> distribution_types{
            { "DEFAULT", static_cast<int>(DistributionAnalysisType::DEFAULT) },
            { "PCA2D", static_cast<int>(DistributionAnalysisType::PCA2D) },
            { "PCA3D", static_cast<int>(DistributionAnalysisType::PCA3D) },
    };

    parameters.addParameter(param_distribution,
                            distribution_enabled_);
    parameters.addConditionalParameter(param::ParameterFactory::declareParameterSet("filter/distribution/type",
                                                                                    distribution_types,
                                                                                    static_cast<int>(DistributionAnalysisType::DEFAULT)),
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
            { "CIE76", static_cast<int>(ColorDifferenceType::CIE76) },
            { "CIE94Grahpics", static_cast<int>(ColorDifferenceType::CIE94Grahpics) },
            { "CIE94Textiles", static_cast<int>(ColorDifferenceType::CIE94Textiles) },
    };

    parameters.addParameter(param_color,
                            color_enabled_);
    parameters.addConditionalParameter(param::ParameterFactory::declareParameterSet("filter/color/type",
                                                                                    color_types,
                                                                                    static_cast<int>(ColorDifferenceType::CIE76)),
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
    out_voxels_            = node_modifier.addOutput<PointCloudMessage>("Voxels");
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
        using Data = VoxelData<DistributionFeature, ColorFeature>;
        selectStorage<Data, PointT>(cloud);
    }
    else if (distribution_enabled_)
    {
        using Data = VoxelData<DistributionFeature>;
        selectStorage<Data, PointT>(cloud);
    }
    else if (color_enabled_)
    {
        using Data = VoxelData<ColorFeature>;
        selectStorage<Data, PointT>(cloud);
    }
    else
    {
        using Data = VoxelData<>;
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
        {
            using Storage = cis::AutoIndexStorage<DataType, cis::backend::simple::UnorderedComponentMap>;
            return clusterCloud<Storage, PointT>(cloud);
        }
        case BackendType::KDTREE:
        {
            using Storage = cis::AutoIndexStorage<DataType, cis::backend::kdtree::KDTreeBuffered>;
            return clusterCloud<Storage, PointT>(cloud);
        }
        case BackendType::ARRAY:
        {
            using Storage = cis::AutoIndexStorage<cis::interface::non_owning<DataType>, cis::backend::array::Array>;
            return clusterCloud<Storage, PointT>(cloud);
        }
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
    using BackendTraits = cis::backend::backend_traits<typename Storage::backend_tag>;
    using Clusterer = ClusterOperation<Storage, DistributionValidator<DataType>, ColorValidator<DataType>>;

    std::vector<DataType> offsite_storage;
    Storage storage;
    {
        NAMED_INTERLUDE(fill_voxel_grid);
        VoxelIndex indexer(voxel_size_[0], voxel_size_[1], voxel_size_[2]);
        StorageOperation::init(*cloud, input_indices, indexer, storage,
                               offsite_storage, std::integral_constant<bool, BackendTraits::IsFixedSize>{});
    }

    if (voxel_validation_enabled_)
    {
        NAMED_INTERLUDE(validate_voxels);
        storage.traverse([this](const VoxelIndex::Type&, DataType& data)
                         {
                             std::size_t min_count = static_cast<std::size_t>(voxel_validation_min_count_);
                             if (voxel_validation_scale_ > 0.0) {
                                 const double depth = data.depth.getMean();
                                 min_count = static_cast<std::size_t>(voxel_validation_min_count_
                                                                      * std::floor(1.0
                                                                                   / (voxel_validation_scale_ * depth * depth)
                                                                                   + 0.5));
                             }
                             if (data.indices.size() < min_count)
                                 data.state = VoxelState::INVALID;
                         });
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
        NAMED_INTERLUDE(extract_clusters);
        StorageOperation::extract(storage,
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

    if (msg::isConnected(out_voxels_))
    {
        NAMED_INTERLUDE(voxel_cloud);

        using VoxelCloud = pcl::PointCloud<pcl::PointXYZRGB>;
        auto voxel_cloud = boost::make_shared<VoxelCloud>();

        auto default_color = Color(50, 65, 75);
        std::map<std::size_t, Color> colors;

        VoxelIndex::Type min_index;
        min_index.fill(std::numeric_limits<int>::max());

        storage.traverse([&min_index](const VoxelIndex::Type& index, const DataType&)
                         {
                             for (std::size_t i = 0; i < 3; ++i)
                                 min_index[i] = std::min(min_index[i], index[i]);
                         });

        auto create_point = [min_index, this](const VoxelIndex::Type& index)
        {
            pcl::PointXYZRGB point;
            point.x = static_cast<float>((index[0] + min_index[0]) * voxel_size_[0] + voxel_size_[0] * 0.5);
            point.y = static_cast<float>((index[1] + min_index[1]) * voxel_size_[1] + voxel_size_[1] * 0.5);
            point.z = static_cast<float>((index[2] + min_index[2]) * voxel_size_[2] + voxel_size_[2] * 0.5);
            return point;
        };

        storage.traverse([&voxel_cloud, &colors, default_color, create_point](const VoxelIndex::Type& index, const DataType& data)
                         {
                             auto point = create_point(index);
                             const auto cluster = data.cluster;

                             if (colors.find(cluster) == colors.end())
                             {
                                 double r = default_color.r, g = default_color.g, b = default_color.b;
                                 if (data.state == VoxelState::ACCEPTED)
                                     color::fromCount(cluster + 1, r, g, b);
                                 colors.emplace(cluster, Color(r, g, b));
                             }

                             const auto& color = colors.at(cluster);
                             point.r = color.r;
                             point.g = color.g;
                             point.b = color.b;
                             voxel_cloud->push_back(point);
                         });

        auto voxel_message = std::make_shared<PointCloudMessage>(cloud->header.frame_id, cloud->header.stamp);
        voxel_message->value = std::move(voxel_cloud);
        msg::publish(out_voxels_, voxel_message);
    }

    msg::publish<GenericVectorMessage, pcl::PointIndices>(out_clusters_accepted_, clusters_accepted_message_);
    msg::publish<GenericVectorMessage, pcl::PointIndices>(out_clusters_rejected_, clusters_rejected_message_);
}

CSAPEX_REGISTER_CLASS(csapex::clustering::ClusterPointCloud, csapex::Node)
