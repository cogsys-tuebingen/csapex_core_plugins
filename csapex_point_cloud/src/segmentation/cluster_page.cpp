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

#include <cslibs_kdtree/page.hpp>
#include <cslibs_kdtree/index.hpp>
#include <cslibs_kdtree/fill.hpp>

CSAPEX_REGISTER_CLASS(csapex::ClusterPointCloudPaging, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace impl {
using DataIndex           = std::array<int, 3>;

struct Entry {
    int                      cluster;
    impl::DataIndex          index;
    std::vector<int>         indices;

    Entry() :
        cluster(-1)
    {
    }
};

using PageType            = kdtree::Page<Entry*, 3>;
using PageIndex           = PageType::Index;
using AO                  = kdtree::ArrayOperations<3, int, int>;
using AOA                 = kdtree::ArrayOperations<3, int, std::size_t>;

struct Indexation {
    using BinType = std::array<double, 3>;

    BinType bin_sizes;

    Indexation(const BinType &_bin_sizes) :
        bin_sizes(_bin_sizes)
    {
    }

    inline static PageType::Size size(const DataIndex &min_index,
                                       const DataIndex &max_index)
    {
        PageType::Size size;
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            size[i] = (max_index[i] - min_index[i]) + 1;
        }
        return size;
    }

    template<typename PointT>
    inline constexpr bool is_valid(const PointT& point) const
    {
        bool nan = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
        bool inf = std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z);
        return !(nan || inf);
    }

    template<typename PointT>
    inline constexpr DataIndex create(const PointT& point) const
    {
        return { static_cast<int>(std::floor(point.x / bin_sizes[0])),
                 static_cast<int>(std::floor(point.y / bin_sizes[1])),
                 static_cast<int>(std::floor(point.z / bin_sizes[2]))};
    }
};

class PageClustering {
public:
    typedef kdtree::detail::fill<DataIndex, 3>   MaskFiller;
    typedef typename MaskFiller::Type            MaskType;

    PageClustering(std::vector<Entry*>            &_entries,
                    std::vector<pcl::PointIndices> &_indices,
                    PageType                      &_array,
                    DataIndex                      &_min_index,
                    DataIndex                      &_max_index) :
        cluster_count(0),
        entries(_entries),
        indices(_indices),
        page(_array),
        min_index(_min_index),
        max_index(_max_index)
    {
        MaskFiller::assign(offsets);
    }

    inline void cluster()
    {
        for(Entry *entry : entries)
        {
            if(entry->cluster > -1)
                continue;
            entry->cluster = cluster_count;
            indices.emplace_back(pcl::PointIndices());
            indices.back().indices = entry->indices;
            ++cluster_count;
            clusterEntry(entry);
        }
    }

private:
    MaskType offsets;
    int      cluster_count;

    std::vector<Entry*>            &entries;
    std::vector<pcl::PointIndices> &indices;
    PageType                      &page;
    DataIndex                      min_index;
    DataIndex                      max_index;

    inline void clusterEntry(Entry *entry)
    {
        PageIndex page_index;
        DataIndex index;
        for(DataIndex &offset : offsets) {
            if(AO::is_zero(offset))
                continue;

            AO::add(entry->index, offset, index);

            bool out_of_bounds = false;
            for(std::size_t j = 0 ; j < 3 ; ++j) {
                out_of_bounds |= index[j] < min_index[j];
                out_of_bounds |= index[j] > max_index[j];
                page_index[j]  = index[j] - min_index[j];
            }

            if(out_of_bounds)
                continue;

            Entry *neighbour = page.at(page_index);
            if(!neighbour)
                continue;
            if(neighbour->cluster > -1)
                continue;
            assert(neighbour->cluster == -1);
            const int cluster = entry->cluster;
            neighbour->cluster = cluster;
            pcl::PointIndices &pcl_indices = indices[cluster];
            pcl_indices.indices.insert(pcl_indices.indices.end(),
                                       neighbour->indices.begin(),
                                       neighbour->indices.end());
            clusterEntry(neighbour);
        }
    }
};


}

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
    impl::DataIndex  min_index = impl::AO::max();
    impl::DataIndex  max_index = impl::AO::min();
    impl::Indexation indexation(cluster_params_.bin_sizes);

    std::vector<impl::Entry>  entries;
    {
        /// Preparation of indices
        if(indices) {
            for(const int i : indices->indices) {
                const PointT &pt = cloud->at(i);
                if(indexation.is_valid(pt)) {
                    impl::Entry entry;
                    entry.index = indexation.create(pt);
                    entry.indices.push_back(i);
                    impl::AO::cwise_min(entry.index, min_index);
                    impl::AO::cwise_max(entry.index, max_index);
                    entries.emplace_back(entry);
                }
            }
        } else {
            const std::size_t cloud_size = cloud->size();
            for(std::size_t i = 0; i < cloud_size ; ++i) {
                const PointT &pt = cloud->at(i);
                if(indexation.is_valid(pt)) {
                    impl::Entry entry;
                    entry.index = indexation.create(pt);
                    entry.indices.push_back(i);
                    impl::AO::cwise_min(entry.index, min_index);
                    impl::AO::cwise_max(entry.index, max_index);
                    entries.emplace_back(entry);
                }
            }
        }
    }
    std::vector<impl::Entry*> referenced;
    impl::PageType::Size size = impl::Indexation::size(min_index, max_index);
    impl::PageType page(size);
    {
        /// Setup array adressing
        impl::PageType::Index index;
        for(impl::Entry &e : entries) {
            index = impl::AOA::sub(e.index, min_index);
            impl::Entry *& page_entry = page.at(index);
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
        impl::PageClustering clustering(referenced, buffer, page, min_index, max_index);
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
