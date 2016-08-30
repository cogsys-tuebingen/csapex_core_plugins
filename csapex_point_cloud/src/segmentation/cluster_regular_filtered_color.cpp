#include "cluster_regular_filtered_color.h"

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
#include "regular_structures/filtered_clustering_color.hpp"
#include <iostream>

#include <cslibs_kdtree/array.hpp>
#include <cslibs_kdtree/page.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using EntryType   = EntryStatisticalColor;

template<typename StructureType>
void ClusterRegularFilteredColor<StructureType>::setup(NodeModifier &node_modifier)
{
    in_cloud_   = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_color_      = node_modifier.addInput<CvMatMessage>("LAB");
    in_indices_ = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
    out_rejected_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Rejected Clusters");
}

template<typename StructureType>
void ClusterRegularFilteredColor<StructureType>::setupParameters(Parameterizable &parameters)
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
    std::map<std::string, int> color_difference_types =
    {
        {"CIE76", ClusterParamsStatisticalColor::CIE76},
        {"CIE94Grahpics", ClusterParamsStatisticalColor::CIE94Grahpics},
        {"CIE94Textiles", ClusterParamsStatisticalColor::CIE94Textiles},
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("cluster/color_difference_type",
                                                                         color_difference_types,
                                                                         (int) ClusterParamsStatisticalColor::CIE76),
                            (int&) cluster_params_.color_difference_type);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/color_difference/max", 0.0, 4000.0, 0.0, 0.1),
                            cluster_params_.color_difference);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/color_difference/weights/l", 0.0, 1.0, 1.0, 0.01),
                            cluster_params_.color_difference_weights[0]);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/color_difference/weights/a", 0.0, 1.0, 1.0, 0.01),
                            cluster_params_.color_difference_weights[1]);
    parameters.addParameter(param::ParameterFactory::declareRange("cluster/color_difference/weights/b", 0.0, 1.0, 1.0, 0.01),
                            cluster_params_.color_difference_weights[2]);
}

/// TODO: Maybe use distribution instead of mean to omit entries of too high variance

template<typename StructureType>
void ClusterRegularFilteredColor<StructureType>::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<ClusterRegularFilteredColor>(this, msg), msg->value);
}

template<typename StructureType>
template <class PointT>
void ClusterRegularFilteredColor<StructureType>::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if (cloud->empty())
        return;

    pcl::PointIndicesPtr indices;
    if(msg::isConnected(in_indices_))
    {
        auto indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
        indices = indices_msg->value;
    }
    CvMatMessage::ConstPtr in_color = msg::getMessage<CvMatMessage>(in_color_);
    const cv::Mat &color = in_color->value;
    if(!in_color->getEncoding().matches(enc::lab)) {
        throw std::runtime_error("Encoding must be enc::lab!");
    }
    if(cloud->height != color.rows ||
            cloud->width != color.cols) {
        throw std::runtime_error("Matrix and point cloud size must match!");
    }

    std::shared_ptr<std::vector<pcl::PointIndices>> out_cluster_indices(new std::vector<pcl::PointIndices>);
    std::shared_ptr<std::vector<pcl::PointIndices>> out_rejected_cluster_indices(new std::vector<pcl::PointIndices>);
    DataIndex  min_index = AO::max();
    DataIndex  max_index = AO::min();
    IndexationType indexation(cluster_params_.bin_sizes);

    std::vector<EntryType>  entries;
    const uchar *color_ptr = color.ptr<uchar>();
    {
        /// Preparation of indices
        if(indices) {
            for(const int i : indices->indices) {
                const PointT &pt = cloud->at(i);
                if(indexation.is_valid(pt)) {
                    EntryType entry;
                    entry.index = indexation.create(pt);
                    entry.indices.push_back(i);
                    entry.distribution.add({pt.x, pt.y, pt.z});
                    const std::size_t color_pos = 3 * i;
                    entry.color_mean.add({(double) color_ptr[color_pos],
                                          (double) color_ptr[color_pos + 1],
                                          (double) color_ptr[color_pos + 2]});
                    AO::cwise_min(entry.index, min_index);
                    AO::cwise_max(entry.index, max_index);
                    entries.emplace_back(entry);
                }
            }
        } else {
            const std::size_t cloud_size = cloud->size();
            for(std::size_t i = 0; i < cloud_size ; ++i) {
                const PointT &pt = cloud->at(i);
                if(indexation.is_valid(pt)) {
                    EntryType entry;
                    entry.index = indexation.create(pt);
                    entry.indices.push_back(i);
                    entry.distribution.add({pt.x, pt.y, pt.z});
                    const std::size_t color_pos = 3 * i;
                    entry.color_mean.add({(double) color_ptr[color_pos],
                                          (double) color_ptr[color_pos + 1],
                                          (double) color_ptr[color_pos + 2]});
                    AO::cwise_min(entry.index, min_index);
                    AO::cwise_max(entry.index, max_index);
                    entries.push_back(entry);
                }
            }
        }
    }
    std::vector<EntryType*> referenced;
    typename StructureType::Size size = IndexationType::size(min_index, max_index);
    StructureType array(size);
    {
        /// Setup array adressing
        typename StructureType::Index index;
        for(EntryType &e : entries) {
            index = AOA::sub(e.index, min_index);
            EntryType *& array_entry = array.at(index);
            if(!array_entry) {
                /// put into array
                array_entry = &e;
                referenced.emplace_back(array_entry);
            } else {
                /// fuse with existing
                std::vector<int> &array_entry_indices = array_entry->indices;
                array_entry_indices.insert(array_entry_indices.end(),
                                           e.indices.begin(),
                                           e.indices.end());
                array_entry->distribution += e.distribution;
                array_entry->color_mean   += e.color_mean;
            }
        }
    }
    {
        /// Clustering stage
        FilteredClusteringColor<StructureType> clustering(referenced,
                                                          cluster_params_,
                                                          *out_cluster_indices,
                                                          *out_rejected_cluster_indices,
                                                          array,
                                                          min_index,
                                                          max_index);
        clustering.cluster();
    }

    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, out_cluster_indices);
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_rejected_, out_rejected_cluster_indices);
}


using PageType    = kdtree::Page<EntryType*, 3>;
using ArrayType   = kdtree::Array<EntryType*, 3>;
namespace csapex {
typedef ClusterRegularFilteredColor<PageType>  ClusterPointCloudPagingFilteredColor;
typedef ClusterRegularFilteredColor<ArrayType> ClusterPointCloudArrayFilteredColor;
}

CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloudPagingFilteredColor, csapex::Node)
CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloudArrayFilteredColor, csapex::Node)
