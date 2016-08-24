/// HEADER
#include "cluster_page.h"

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

#include <cslibs_kdtree/fill.hpp>
#include <cslibs_kdtree/page.hpp>
#include <cslibs_kdtree/index.hpp>

#include "regular_structures/indexation.hpp"
#include "regular_structures/clustering.hpp"

CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloudPaging, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

using PageType           = kdtree::Page<Entry*, 3>;
using PageIndex          = PageType::Index;
using IndexationType     = Indexation<PageType>;

void ClusterPointCloudPaging::setup(NodeModifier &node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_ = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
}

void ClusterPointCloudPaging::setupParameters(Parameterizable &parameters)
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

void ClusterPointCloudPaging::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<ClusterPointCloudPaging>(this, msg), msg->value);
}

template <class PointT>
void ClusterPointCloudPaging::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
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
    DataIndex      min_index = AO::max();
    DataIndex      max_index = AO::min();
    IndexationType indexation(cluster_params_.bin_sizes);

    std::vector<Entry>  entries;
    {
        /// Preparation of indices
        if(indices) {
            for(const int i : indices->indices) {
                const PointT &pt = cloud->at(i);
                if(indexation.is_valid(pt)) {
                    Entry entry;
                    entry.index = indexation.create(pt);
                    entry.indices.push_back(i);
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
                    Entry entry;
                    entry.index = indexation.create(pt);
                    entry.indices.push_back(i);
                    AO::cwise_min(entry.index, min_index);
                    AO::cwise_max(entry.index, max_index);
                    entries.emplace_back(entry);
                }
            }
        }
    }
    std::vector<Entry*> referenced;
    PageType::Size size = IndexationType::size(min_index, max_index);
    PageType page(size);
    {
        /// Setup array adressing
        PageType::Index index;
        for(Entry &e : entries) {
            index = AOA::sub(e.index, min_index);
            Entry *& page_entry = page.at(index);
            if(!page_entry) {
                /// put into array
                page_entry = &e;
                referenced.emplace_back(page_entry);
            } else {
                /// fuse with existing
                std::vector<int> &array_entry_indices = page_entry->indices;
                array_entry_indices.insert(array_entry_indices.end(),
                                           e.indices.begin(),
                                           e.indices.end());
            }
        }
    }
    {
        /// Clustering stage
        std::vector<pcl::PointIndices> buffer;
        Clustering<PageType> clustering(referenced, buffer, page, min_index, max_index);
        clustering.cluster();

        for(pcl::PointIndices &pcl_indices : buffer) {
            if(pcl_indices.indices.size() >= cluster_params_.cluster_sizes[0] &&
                    pcl_indices.indices.size() <= cluster_params_.cluster_sizes[1]) {
                out_cluster_indices->push_back(pcl_indices);
            }
        }
    }
    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, out_cluster_indices);
}
