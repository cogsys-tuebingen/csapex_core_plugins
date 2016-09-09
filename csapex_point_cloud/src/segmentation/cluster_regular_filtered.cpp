#include "cluster_regular_filtered.h"

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

#include "regular_structures/indexation.hpp"
#include "regular_structures/filtered_clustering.hpp"

#include <cslibs_kdtree/array.hpp>
#include <cslibs_kdtree/page.hpp>

using namespace csapex;
using namespace csapex::connection_types;

template<typename StructureType>
void ClusterRegularFiltered<StructureType>::setup(NodeModifier &node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_ = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
    out_rejected_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Rejected Clusters");
}

template<typename StructureType>
void ClusterRegularFiltered<StructureType>::setupParameters(Parameterizable &parameters)
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
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/voxel/min_points", 0, 1000000, 0, 1),
                            cluster_params_.voxel_min_points);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/voxel/min_points_scale", 0.0, 1.0, 0.0, 0.001),
                            cluster_params_.voxel_min_points_scale);

    std::map<std::string, int> covariance_threshold_types = {{"DEFAULT", ClusterParamsStatistical::DEFAULT},
                                                             {"PCA2D", ClusterParamsStatistical::PCA2D},
                                                             {"PCA3D", ClusterParamsStatistical::PCA3D}};
    parameters.addParameter(param::ParameterFactory::declareParameterSet("cluster/std_dev_thresh_type",
                                                                         covariance_threshold_types,
                                                                         (int) ClusterParamsStatistical::DEFAULT),
                            reinterpret_cast<int&>(cluster_params_.cluster_cov_thresh_type));
}

template<typename StructureType>
void ClusterRegularFiltered<StructureType>::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<ClusterRegularFiltered>(this, msg), msg->value);
}

template<typename StructureType>
template <class PointT>
void ClusterRegularFiltered<StructureType>::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if (cloud->empty())
        return;

    pcl::PointIndicesPtr indices;
    if(msg::isConnected(in_indices_))
    {
        auto indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
        indices = indices_msg->value;
    }

    std::shared_ptr<std::vector<pcl::PointIndices>> out_cluster_indices(new std::vector<pcl::PointIndices>);
    std::shared_ptr<std::vector<pcl::PointIndices>> out_rejected_cluster_indices(new std::vector<pcl::PointIndices>);
    DataIndex  min_index = AO::max();
    DataIndex  max_index = AO::min();
    IndexationType indexation(cluster_params_.bin_sizes);

    std::vector<EntryStatistical>  entries;
    {
        /// Preparation of indices
        if(indices) {
            const int *indices_ptr = indices->indices.data();
            const std::size_t size = indices->indices.size();
            const PointT *points_ptr = cloud->points.data();

            entries.resize(size);
            EntryStatistical *entries_ptr = entries.data();
            for(std::size_t i = 0 ; i < size ; ++i, ++indices_ptr) {
                const int index = *indices_ptr;
                const PointT &pt = points_ptr[index];
                if(indexation.is_valid(pt)) {
                    EntryStatistical &entry = entries_ptr[i];
                    entry.valid = true;
                    entry.index = indexation.create(pt);
                    entry.indices.push_back(index);
                    entry.distribution.add({pt.x, pt.y, pt.z});
                    entry.depth_mean.add(sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z));
                    AO::cwise_min(entry.index, min_index);
                    AO::cwise_max(entry.index, max_index);
                }
            }
        } else {
            const std::size_t cloud_size = cloud->size();
            const PointT *points_ptr = cloud->points.data();

            entries.resize(cloud_size);
            EntryStatistical *entries_ptr = entries.data();
            for(std::size_t i = 0; i < cloud_size ; ++i) {
                const PointT &pt = points_ptr[i];
                if(indexation.is_valid(pt)) {
                    EntryStatistical &entry = entries_ptr[i];
                    entry.valid = true;
                    entry.index = indexation.create(pt);
                    entry.indices.push_back(i);
                    entry.distribution.add({pt.x, pt.y, pt.z});
                    entry.depth_mean.add(sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z));
                    AO::cwise_min(entry.index, min_index);
                    AO::cwise_max(entry.index, max_index);
                }
            }
        }
    }

    std::vector<EntryStatistical*> referenced;
    typename StructureType::Size size = IndexationType::size(min_index, max_index);
    StructureType array(size);
    {
        /// Setup array adressing
        typename StructureType::Index index;
        EntryStatistical *entries_ptr = entries.data();
        const std::size_t size = entries.size();
        for(std::size_t i = 0 ; i < size ; ++i, ++entries_ptr) {
            EntryStatistical &e = *entries_ptr;

            if(!e.valid)
                continue;

            index = AOA::sub(e.index, min_index);
            EntryStatistical *& array_entry = array.at(index);
            if(!array_entry) {
                /// put into array
                array_entry = &e;
                validateVoxel(*array_entry);
                referenced.emplace_back(array_entry);
            } else {
                /// fuse with existing
                std::vector<int> &array_entry_indices = array_entry->indices;
                array_entry_indices.insert(array_entry_indices.end(),
                                           e.indices.begin(),
                                           e.indices.end());
                array_entry->distribution += e.distribution;
                array_entry->depth_mean   += e.depth_mean;
                validateVoxel(*array_entry);
            }
        }
    }
    {
        /// Clustering stage
        FilteredClustering<StructureType> clustering(referenced, cluster_params_, *out_cluster_indices, *out_rejected_cluster_indices, array, min_index, max_index);
        clustering.cluster();
    }
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, out_cluster_indices);
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_rejected_, out_rejected_cluster_indices);
}

template<typename StructureType>
void ClusterRegularFiltered<StructureType>::validateVoxel(EntryStatistical &e)
{
    if(cluster_params_.voxel_min_points) {
        std::size_t min_count = cluster_params_.voxel_min_points;
        if(cluster_params_.voxel_min_points_scale > 0.0) {
            const double depth = e.depth_mean.getMean();
            min_count = cluster_params_.voxel_min_points * floor(1.0 / (cluster_params_.voxel_min_points_scale * depth * depth) + 0.5);
        }
        e.valid = e.distribution.getN() >= min_count;
    }
}


using PageType    = kdtree::Page<EntryStatistical*, 3>;
using ArrayType   = kdtree::Array<EntryStatistical*, 3>;
namespace csapex {
typedef ClusterRegularFiltered<PageType>  ClusterPointCloudPagingFiltered;
typedef ClusterRegularFiltered<ArrayType> ClusterPointCloudArrayFiltered;
}

CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloudPagingFiltered, csapex::Node)
CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloudArrayFiltered, csapex::Node)
