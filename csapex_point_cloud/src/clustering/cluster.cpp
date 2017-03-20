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
/// Used tight representation during voxel visualization
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
    // backend config
    static const std::map<std::string, int> backend_types{
            { "page", static_cast<int>(BackendType::PAGED) },
            { "k-d tree", static_cast<int>(BackendType::KDTREE) },
            { "array", static_cast<int>(BackendType::ARRAY) },
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("backend",
                                                                         param::ParameterDescription("Backend used for clustering. Available methods:"
                                                                                                     "<ul>"
                                                                                                     "<li>Page: by-axis nested maps</li>"
                                                                                                     "<li>k-d tree: unbalanced k-d tree</li>"
                                                                                                     "<li>array: pre-allocated array, with O(1) access</li>"
                                                                                                     "</ul>"),
                                                                         backend_types,
                                                                         static_cast<int>(BackendType::PAGED)),
                            reinterpret_cast<int&>(backend_));


    // general clustering config
    parameters.addParameter(param::ParameterFactory::declareRange("voxel/size/x",
                                                                  param::ParameterDescription("Voxel size (in m) (along x-axis)"),
                                                                  0.01, 10.0, 0.1, 0.01),
                            voxel_size_[0]);
    parameters.addParameter(param::ParameterFactory::declareRange("voxel/size/y",
                                                                  param::ParameterDescription("Voxel size (in m) (along y-axis)"),
                                                                  0.01, 10.0, 0.1, 0.01),
                            voxel_size_[1]);
    parameters.addParameter(param::ParameterFactory::declareRange("voxel/size/z",
                                                                  param::ParameterDescription("Voxel size (in m) (along z-axis)"),
                                                                  0.01, 10.0, 0.1, 0.01),
                            voxel_size_[2]);


    parameters.addParameter(param::ParameterFactory::declareInterval("filter/point_count",
                                                                     param::ParameterDescription("Filter by points within final clusters,"
                                                                                                 " e.g. to reject small noisy clustering or extremly large ones."),
                                                                     1, 10000000, 1, 10000000, 1),
                           cluster_point_count_);


    // voxel pre-clustering validation
    param::ParameterPtr param_voxel_validation
            = param::ParameterFactory::declareBool("filter/voxel_validation",
                                                   param::ParameterDescription("Pre-filter voxels by point count, optionally scaled by distance."
                                                                               " Allows to remove voxels which are likely sensor noise."),
                                                   false);
    std::function<bool()> enable_voxel_validation = [param_voxel_validation]() -> bool { return param_voxel_validation->as<bool>(); };

    parameters.addParameter(param_voxel_validation,
                            voxel_validation_enabled_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/voxel_validation/min_count",
                                                                             param::ParameterDescription("Minimal point count in a valid voxel"),
                                                                             1, 1000, 1, 1),
                                       enable_voxel_validation,
                                       voxel_validation_min_count_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/voxel_validation/scale",
                                                                             param::ParameterDescription("Scale min count with the voxel distance."
                                                                                                         " Allows to adjust non-linear relationship between point denstity and distance."
                                                                                                         "<br>Scale factor: 1 / (scale * depth^2)"),
                                                                             0.0, 1.0, 0.0, 0.001),
                                       enable_voxel_validation,
                                       voxel_validation_scale_);


    // cluster distribution validation
    param::ParameterPtr param_distribution
            = param::ParameterFactory::declareBool("filter/distribution",
                                                   param::ParameterDescription("Filter clusters by their point distribution."
                                                                               " Allows to distinguish between e.g. walls and persons based on their apprearance."),
                                                   false);
    std::function<bool()> enable_distribution = [param_distribution]() -> bool  { return param_distribution->as<bool>(); };
    static const std::map<std::string, int> distribution_types{
            { "DEFAULT", static_cast<int>(DistributionAnalysisType::DEFAULT) },
            { "PCA2D", static_cast<int>(DistributionAnalysisType::PCA2D) },
            { "PCA3D", static_cast<int>(DistributionAnalysisType::PCA3D) },
    };

    parameters.addParameter(param_distribution,
                            distribution_enabled_);
    parameters.addConditionalParameter(param::ParameterFactory::declareParameterSet("filter/distribution/type",
                                                                                    param::ParameterDescription("Distribution analysis type to determine axis. Available methods:"
                                                                                                                "<ul>"
                                                                                                                "<li>default: assume axis alignment</li>"
                                                                                                                "<li>PCA 2D: use 2D PCA analysis, ignores z-axis value</li>"
                                                                                                                "<li>PCA 3D: use 3D PCA analysis</li>"
                                                                                                                "</ul>"),
                                                                                    distribution_types,
                                                                                    static_cast<int>(DistributionAnalysisType::DEFAULT)),
                                       enable_distribution,
                                       reinterpret_cast<int&>(distribution_type_));
    parameters.addConditionalParameter(param::ParameterFactory::declareInterval("filter/distribution/std_dev/x",
                                                                                param::ParameterDescription("Standard Deviation threshold (in m) (along x-axis)."
                                                                                                            " A range of [0,0] deactivates the threshold."),
                                                                                0.0, 10.0, 0.0, 10.0, 0.01),
                                       enable_distribution,
                                       distribution_std_dev_[0]);
    parameters.addConditionalParameter(param::ParameterFactory::declareInterval("filter/distribution/std_dev/y",
                                                                                param::ParameterDescription("Standard Deviation threshold (in m) (along y-axis)."
                                                                                                            " A range of [0,0] deactivates the threshold."),
                                                                                0.0, 10.0, 0.0, 10.0, 0.01),
                                       enable_distribution,
                                       distribution_std_dev_[1]);
    parameters.addConditionalParameter(param::ParameterFactory::declareInterval("filter/distribution/std_dev/z",
                                                                                param::ParameterDescription("Standard Deviation threshold (in m) (along z-axis) (unused when using 2D PCA)"
                                                                                                            " A range of [0,0] deactivates the threshold."),
                                                                                0.0, 10.0, 0.0, 10.0, 0.01),
                                       enable_distribution,
                                       distribution_std_dev_[2]);


    /// cluster by normal
    parameters.addParameter(param::ParameterFactory::declareBool("filter/normal",
                                                                 param::ParameterDescription("Only cluster voxels which have a normal vector parallel to the given one."),
                                                                 false),
                            validate_normal_);
    parameters.addConditionalParameter(param::ParameterFactory::declareValue("filter/normal/x",
                                                                             param::ParameterDescription("x coordinate of the normal vector."),
                                                                             0.0),
                                       [this]{return validate_normal_;},
                                       normal_(0));
    parameters.addConditionalParameter(param::ParameterFactory::declareValue("filter/normal/y",
                                                                             param::ParameterDescription("y coordinate of the normal vector."),
                                                                             0.0),
                                       [this]{return validate_normal_;},
                                       normal_(1));
    parameters.addConditionalParameter(param::ParameterFactory::declareValue("filter/normal/z",
                                                                             param::ParameterDescription("z coordinate of the normal vector."),
                                                                             1.0),
                                       [this]{return validate_normal_;},
                                       normal_(2));
    parameters.addConditionalParameter(param::ParameterFactory::declareAngle("filter/normal/maximum_angle_difference",
                                                                             param::ParameterDescription("Maximum angle difference to normal."),
                                                                             0.0),
                                       [this]{return validate_normal_;},
                                       normal_maximum_angle_distance_);


    // cluster color validation
    param::ParameterPtr param_color
            = param::ParameterFactory::declareBool("filter/color",
                                                   param::ParameterDescription("Filter considered neighboring voxels by their color."
                                                                               " Allows to distinguish persons near walls if the general color appearance is distinct enough."
                                                                               " Works for greyscale and color data."
                                                                               " Uses absolute grayscale value difference in the grayscale case"
                                                                               " or the selected color difference method for color data."),
                                                   false);
    std::function<bool()> enable_color = [param_color]() -> bool { return param_color->as<bool>(); };
    static const std::map<std::string, int> color_types{
            { "CIE76", static_cast<int>(ColorDifferenceType::CIE76) },
            { "CIE94Grahpics", static_cast<int>(ColorDifferenceType::CIE94Grahpics) },
            { "CIE94Textiles", static_cast<int>(ColorDifferenceType::CIE94Textiles) },
    };

    parameters.addParameter(param_color,
                            color_enabled_);
    parameters.addConditionalParameter(param::ParameterFactory::declareParameterSet("filter/color/type",
                                                                                    param::ParameterDescription("Methods used to determine the color difference."
                                                                                                                " Uses the LAB color-space."),
                                                                                    color_types,
                                                                                    static_cast<int>(ColorDifferenceType::CIE76)),
                                       enable_color,
                                       reinterpret_cast<int&>(color_type_));
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/color/max_difference",
                                                                             param::ParameterDescription("Maximum allowed color difference."),
                                                                             0.0, 4000.0, 0.0, 0.1),
                                       enable_color,
                                       color_threshold_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/color/weights/l",
                                                                             param::ParameterDescription("Weighting factor for the l-channel"),
                                                                             0.0, 1.0, 1.0, 0.01),
                                       enable_color,
                                       color_weights_[0]);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/color/weights/a",
                                                                             param::ParameterDescription("Weighting factor for the a-channel"),
                                                                             0.0, 1.0, 1.0, 0.01),
                                       enable_color,
                                       color_weights_[1]);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/color/weights/b",
                                                                             param::ParameterDescription("Weighting factor for the b-channel"),
                                                                             0.0, 1.0, 1.0, 0.01),
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
    /* When adding a new filter:
     * - Define feature, add parameters, create validator (inclusive no-op fallback)
     * - Add combination cases below
     *
     * TODO: find better method for dynamic -> static type dispatch
     */

    if (distribution_enabled_ && color_enabled_)
    {
        using Data = VoxelData<DistributionFeature, ColorFeature>;
        selectStorage<Data, PointT>(cloud);
    }
    else if (distribution_enabled_ || validate_normal_)
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
    /* When adding new types:
     * - define and add BackendType value and add description
     * - if dynamic sized: use dense/default storage hint
     * - if fixed sized  : use non_owning storage hint, data is stored elsewhere to allow size precomputation
     */
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
    // no cloud -> nothing todo
    if (cloud->empty())
        return;

    // allocate/extract messages
    pcl::PointIndices::ConstPtr input_indices;
    if (msg::isConnected(in_indices_))
    {
        auto indices_message = msg::getMessage<PointIndecesMessage>(in_indices_);
        input_indices = indices_message->value;
    }

    auto clusters_accepted_message_ = std::make_shared<std::vector<pcl::PointIndices>>();
    auto clusters_rejected_message_ = std::make_shared<std::vector<pcl::PointIndices>>();

    // define storage & clustering types types
    using DataType = typename Storage::data_t;
    using BackendTraits = cis::backend::backend_traits<typename Storage::backend_tag>;
    using Clusterer = ClusterOperation<Storage, DistributionValidator<DataType>, ColorValidator<DataType>>;

    // offsite storage is only used for fixed storage types where we have to precompute the size
    std::vector<DataType> offsite_storage;
    Storage storage;
    {
        NAMED_INTERLUDE(fill_voxel_grid);

        // create indexer and fill storage
        VoxelIndex indexer(voxel_size_[0], voxel_size_[1], voxel_size_[2]);
        StorageOperation::init(*cloud, input_indices, indexer, storage,
                               offsite_storage, std::integral_constant<bool, BackendTraits::IsFixedSize>{});
    }

    // pre-filter voxel based on point count (if enabled)
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
    if(validate_normal_) {
        NAMED_INTERLUDE(validate_normals);

        storage.traverse([this](const VoxelIndex::Type&, DataType& data)
                         {


//                            auto& feature = data.template getFeature<DistributionFeature>();

//                            data.state = VoxelState::INVALID;
                         });

        /// put in distribution here, consult pcl for nomal estimation with pca

    }


    // register feature validators
    // - validators default to no-op (aka. always true) if feature is not selected
    // - add your validators for new features here
    DistributionValidator<DataType> distribution_validator(distribution_type_,
                                                           distribution_std_dev_);
    ColorValidator<DataType> color_validator(color_type_,
                                             color_weights_,
                                             color_threshold_);

    // define the cluster operation
    Clusterer cluster_op(storage,
                         distribution_validator,
                         color_validator);
    {
        NAMED_INTERLUDE(cluster);

        // run the clustering
        cis::operations::clustering::Clustering<Storage> clustering(storage);
        clustering.cluster(cluster_op);
    }

    {
        NAMED_INTERLUDE(extract_clusters);

        // extract point cloud indices for each cluster
        StorageOperation::extract(storage,
                            cluster_op,
                            *clusters_accepted_message_,
                            *clusters_rejected_message_);
    }

    {
        NAMED_INTERLUDE(count_filter);

        // filter clusters by size
        // we consider invalid sizes as "not-a-cluster" and not as "rejected/invalid"
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

    // if requested visualize clusters and voxels
    if (msg::isConnected(out_voxels_))
    {
        NAMED_INTERLUDE(voxel_cloud);

        using VoxelCloud = pcl::PointCloud<pcl::PointXYZRGB>;
        auto voxel_cloud = boost::make_shared<VoxelCloud>();

        // color map -> each cluster has distinct color
        auto default_color = Color(50, 65, 75);
        std::map<std::size_t, Color> colors;

        // find the minimal index to correctly align cloud
        VoxelIndex::Type min_index;
        min_index.fill(std::numeric_limits<int>::max());

        storage.traverse([&min_index](const VoxelIndex::Type& index, const DataType&)
                         {
                             for (std::size_t i = 0; i < 3; ++i)
                                 min_index[i] = std::min(min_index[i], index[i]);
                         });

        // projects index to 3D position
        auto create_point = [min_index, this](const VoxelIndex::Type& index)
        {
            pcl::PointXYZRGB point;
            point.x = static_cast<float>((index[0] + min_index[0]) * voxel_size_[0] + voxel_size_[0] * 0.5);
            point.y = static_cast<float>((index[1] + min_index[1]) * voxel_size_[1] + voxel_size_[1] * 0.5);
            point.z = static_cast<float>((index[2] + min_index[2]) * voxel_size_[2] + voxel_size_[2] * 0.5);
            return point;
        };

        // visualize each (valid) voxel, invalid voxels are grey
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
