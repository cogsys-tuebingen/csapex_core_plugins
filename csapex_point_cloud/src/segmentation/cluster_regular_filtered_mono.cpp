#include "cluster_regular_filtered_mono.h"

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
#include <csapex_opencv/cv_mat_message.h>

#include "regular_structures/indexation.hpp"
#include "regular_structures/entry.hpp"
#include "regular_structures/filtered_clustering_mono.hpp"
#include <iostream>

#include <cslibs_kdtree/array.hpp>
#include <cslibs_kdtree/page.hpp>

#include <unordered_set>

using namespace csapex;
using namespace csapex::connection_types;
using EntryType = EntryStatisticalMono;

template<typename StructureType>
void ClusterRegularFilteredMono<StructureType>::setup(NodeModifier &node_modifier)
{
    in_cloud_     = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_mono_      = node_modifier.addInput<CvMatMessage>("Intensity / Grayscale");
    in_indices_   = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_          = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
    out_rejected_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Rejected Clusters");
}

template<typename StructureType>
void ClusterRegularFilteredMono<StructureType>::setupParameters(Parameterizable &parameters)
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

    std::map<std::string, int> covariance_threshold_types = {{"DEFAULT", ClusterParamsStatistical::DEFAULT},
                                                             {"PCA2D", ClusterParamsStatistical::PCA2D},
                                                             {"PCA3D", ClusterParamsStatistical::PCA3D}};
    parameters.addParameter(param::ParameterFactory::declareParameterSet("cluster/std_dev_thresh_type",
                                                                         covariance_threshold_types,
                                                                         (int) ClusterParamsStatistical::DEFAULT),
                            reinterpret_cast<int&>(cluster_params_.cluster_cov_thresh_type));
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/mono_difference/max", 0.0, 4000.0, 0.0, 0.1),
                            cluster_params_.mono_difference);
}

/// TODO: Maybe use distribution instead of mean to omit entries of too high variance

template<typename StructureType>
void ClusterRegularFilteredMono<StructureType>::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<ClusterRegularFilteredMono>(this, msg), msg->value);
}

namespace {

template<typename StructureType,
         class PointT,
         typename T>
inline void prepareEntries(typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                           const cv::Mat                              &mono,
                           const pcl::PointIndicesPtr                 &indices,
                           const ClusterParamsStatisticalMono         &cluster_params,
                           std::vector<EntryType>                     &entries,
                           DataIndex                                  &min_index,
                           DataIndex                                  &max_index)
{
    Indexation<StructureType> indexation(cluster_params.bin_sizes);
    min_index = AO::max();
    max_index = AO::min();
    const T *mono_ptr = mono.ptr<T>();
    for(const int i : indices->indices) {
        const PointT &pt = cloud->at(i);
        if(indexation.is_valid(pt)) {
            EntryType entry;
            entry.valid = true;
            entry.index = indexation.create(pt);
            entry.indices.push_back(i);
            entry.distribution.add({pt.x, pt.y, pt.z});
            entry.mono_mean.add(mono_ptr[i]);
            AO::cwise_min(entry.index, min_index);
            AO::cwise_max(entry.index, max_index);
            entries.emplace_back(entry);
        }
    }
}

template<typename StructureType,
         class PointT,
         typename T>
inline void prepareEntries(typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                           const cv::Mat                              &mono,
                           const ClusterParamsStatisticalMono         &cluster_params,
                           std::vector<EntryType>                     &entries,
                           DataIndex                                  &min_index,
                           DataIndex                                  &max_index)
{
    Indexation<StructureType> indexation(cluster_params.bin_sizes);
    min_index = AO::max();
    max_index = AO::min();
    const T *mono_ptr = mono.ptr<T>();
    const std::size_t cloud_size = cloud->size();
    for(std::size_t i = 0; i < cloud_size ; ++i) {
        const PointT &pt = cloud->at(i);
        if(indexation.is_valid(pt)) {
            EntryType entry;
            entry.valid = true;
            entry.index = indexation.create(pt);
            entry.indices.push_back(i);
            entry.distribution.add({pt.x, pt.y, pt.z});
            entry.mono_mean.add(mono_ptr[i]);
            AO::cwise_min(entry.index, min_index);
            AO::cwise_max(entry.index, max_index);
            entries.push_back(entry);
        }
    }

}


}



template<typename StructureType>
template <class PointT>
void ClusterRegularFilteredMono<StructureType>::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if (cloud->empty())
        return;

    pcl::PointIndicesPtr indices;
    if(msg::isConnected(in_indices_))
    {
        auto indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
        indices = indices_msg->value;
    }
    CvMatMessage::ConstPtr in_mono = msg::getMessage<CvMatMessage>(in_mono_);
    const cv::Mat &mono = in_mono->value;

    if(cloud->height != mono.rows ||
            cloud->width != mono.cols) {
        throw std::runtime_error("Matrix and point cloud size must match!");
    }

    std::shared_ptr<std::vector<pcl::PointIndices>> out_cluster_indices(new std::vector<pcl::PointIndices>);
    std::shared_ptr<std::vector<pcl::PointIndices>> out_rejected_cluster_indices(new std::vector<pcl::PointIndices>);
    DataIndex  min_index = AO::max();
    DataIndex  max_index = AO::min();

    std::vector<EntryType>  entries;
    {
        /// Preparation of indices
        if(indices) {
            switch(mono.type()) {
            case CV_8UC1:
                prepareEntries<StructureType, PointT, uchar>(cloud,
                                                             mono,
                                                             indices,
                                                             cluster_params_,
                                                             entries,
                                                             min_index,
                                                             max_index);
                break;
            case CV_8SC1:
                prepareEntries<StructureType, PointT, char>(cloud,
                                                            mono,
                                                            indices,
                                                            cluster_params_,
                                                            entries,
                                                            min_index,
                                                            max_index);
                break;
            case CV_16UC1:
                prepareEntries<StructureType, PointT, ushort>(cloud,
                                                              mono,
                                                              indices,
                                                              cluster_params_,
                                                              entries,
                                                              min_index,
                                                              max_index);
                break;
            case CV_16SC1:
                prepareEntries<StructureType, PointT, short>(cloud,
                                                             mono,
                                                             indices,
                                                             cluster_params_,
                                                             entries,
                                                             min_index,
                                                             max_index);
                break;
            case CV_32SC1:
                prepareEntries<StructureType, PointT, int>(cloud,
                                                           mono,
                                                           indices,
                                                           cluster_params_,
                                                           entries,
                                                           min_index,
                                                           max_index);
                break;
            case CV_32FC1:
                prepareEntries<StructureType, PointT, float>(cloud,
                                                             mono,
                                                             indices,
                                                             cluster_params_,
                                                             entries,
                                                             min_index,
                                                             max_index);
                break;
            case CV_64FC1:
                prepareEntries<StructureType, PointT, double>(cloud,
                                                              mono,
                                                              indices,
                                                              cluster_params_,
                                                              entries,
                                                              min_index,
                                                              max_index);
                break;
            default:
                throw std::runtime_error("Intensity images may only have one channel.");
            }
        } else {
            switch(mono.type()) {
            case CV_8UC1:
                prepareEntries<StructureType, PointT, uchar>(cloud,
                                                             mono,
                                                             cluster_params_,
                                                             entries,
                                                             min_index,
                                                             max_index);
                break;
            case CV_8SC1:
                prepareEntries<StructureType, PointT, char>(cloud,
                                                            mono,
                                                            cluster_params_,
                                                            entries,
                                                            min_index,
                                                            max_index);
                break;
            case CV_16UC1:
                prepareEntries<StructureType, PointT, ushort>(cloud,
                                                              mono,
                                                              cluster_params_,
                                                              entries,
                                                              min_index,
                                                              max_index);
                break;
            case CV_16SC1:
                prepareEntries<StructureType, PointT, short>(cloud,
                                                             mono,
                                                             cluster_params_,
                                                             entries,
                                                             min_index,
                                                             max_index);
                break;
            case CV_32SC1:
                prepareEntries<StructureType, PointT, int>(cloud,
                                                           mono,
                                                           cluster_params_,
                                                           entries,
                                                           min_index,
                                                           max_index);
                break;
            case CV_32FC1:
                prepareEntries<StructureType, PointT, float>(cloud,
                                                             mono,
                                                             cluster_params_,
                                                             entries,
                                                             min_index,
                                                             max_index);
                break;
            case CV_64FC1:
                prepareEntries<StructureType, PointT, double>(cloud,
                                                              mono,
                                                              cluster_params_,
                                                              entries,
                                                              min_index,
                                                              max_index);
                break;
            default:
                throw std::runtime_error("Intensity images may only have one channel.");
            }
        }
    }

    typename StructureType::Size size = IndexationType::size(min_index, max_index);
    StructureType array(size);
    std::unordered_set<EntryType*> to_check;
    for (auto& entry : entries)
    {
        typename StructureType::Index index;
        index = AOA::sub(entry.index, min_index);
        to_check.emplace(&array.insert(index, entry));
    }
    {
        /// Clustering stage
        FilteredClusteringMono<StructureType> clustering(cluster_params_,
                                                         *out_cluster_indices,
                                                         *out_rejected_cluster_indices,
                                                          array,
                                                          min_index,
                                                          max_index);
        clustering.cluster(to_check.begin(), to_check.end());
    }
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, out_cluster_indices);
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_rejected_, out_rejected_cluster_indices);
}


using PageType    = kdtree::Page<EntryType, 3>;
using ArrayType   = kdtree::Array<EntryType, 3>;
namespace csapex {
typedef ClusterRegularFilteredMono<PageType>  ClusterPointCloudPagingFilteredMono;
typedef ClusterRegularFilteredMono<ArrayType> ClusterPointCloudArrayFilteredMono;
}

CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloudPagingFilteredMono, csapex::Node)
CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloudArrayFilteredMono,  csapex::Node)
