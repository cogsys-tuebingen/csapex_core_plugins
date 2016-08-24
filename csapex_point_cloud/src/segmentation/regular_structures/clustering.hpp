#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <cslibs_kdtree/fill.hpp>
#include <cslibs_kdtree/index.hpp>

#include "entry.hpp"
#include "validator.hpp"

namespace csapex {
using AO                  = kdtree::ArrayOperations<3, int, int>;
using AOA                 = kdtree::ArrayOperations<3, int, std::size_t>;

template<typename StructureType>
class Clustering {
public:
    typedef kdtree::detail::fill<DataIndex, 3>   MaskFiller;
    typedef typename MaskFiller::Type            MaskType;
    typedef typename StructureType::Index        StructureIndex;

    Clustering(std::vector<Entry*>            &_entries,
                    std::vector<pcl::PointIndices> &_indices,
                    StructureType                  &_array,
                    DataIndex                      &_min_index,
                    DataIndex                      &_max_index) :
        cluster_count(0),
        entries(_entries),
        indices(_indices),
        array(_array),
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
    StructureType                  &array;
    DataIndex                       min_index;
    DataIndex                       max_index;

    inline void clusterEntry(Entry *entry)
    {
        StructureIndex array_index;
        DataIndex index;
        for(DataIndex &offset : offsets) {
            if(AO::is_zero(offset))
                continue;

            AO::add(entry->index, offset, index);

            bool out_of_bounds = false;
            for(std::size_t j = 0 ; j < 3 ; ++j) {
                out_of_bounds |= index[j] < min_index[j];
                out_of_bounds |= index[j] > max_index[j];
                array_index[j]  = index[j] - min_index[j];
            }

            if(out_of_bounds)
                continue;

            Entry *neighbour = array.at(array_index);
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

#endif // CLUSTERING_HPP
