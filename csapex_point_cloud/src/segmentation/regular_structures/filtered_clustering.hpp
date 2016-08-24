#ifndef FILTERED_CLUSTERING_HPP
#define FILTERED_CLUSTERING_HPP

#include <cslibs_kdtree/fill.hpp>
#include <cslibs_kdtree/index.hpp>

#include "entry.hpp"
#include "validator.hpp"

namespace csapex {
using AO                  = kdtree::ArrayOperations<3, int, int>;
using AOA                 = kdtree::ArrayOperations<3, int, std::size_t>;

template<typename StructureType>
class FilteredClustering
{
public:
    typedef kdtree::detail::fill<DataIndex, 3>   MaskFiller;
    typedef typename MaskFiller::Type            MaskType;
    typedef typename StructureType::Index        StructureIndex;


    FilteredClustering(std::vector<EntryStatistical*> &_entries,
                   const ClusterParamsStatistical              &_params,
                   std::vector<pcl::PointIndices>   &_indices,
                   std::vector<pcl::PointIndices>   &_indices_rejected,
                   StructureType                    &_array,
                   DataIndex                        &_min_index,
                   DataIndex                        &_max_index) :
        cluster_count(0),
        entries(_entries),
        indices(_indices),
        indices_rejected(_indices_rejected),
        array(_array),
        min_index(_min_index),
        max_index(_max_index),
        validator(_params, buffer_indices, buffer_distribution)
    {
        MaskFiller::assign(offsets);
    }

    inline void cluster()
    {
        for(EntryStatistical *entry : entries)
        {
            if(entry->cluster > -1)
                continue;

            Validator::Result validation = validator.validate();
            if (validation == Validator::Result::ACCEPTED)
                indices.emplace_back(std::move(buffer_indices));
            else
            {
                if (validation != Validator::Result::TOO_SMALL)
                    indices_rejected.emplace_back(std::move(buffer_indices));
                else
                    buffer_indices.indices.clear();
            }

            buffer_distribution.reset();

            entry->cluster = cluster_count;
            ++cluster_count;
            clusterEntry(entry);
        }

        Validator::Result validation = validator.validate();
        if (validation == Validator::Result::ACCEPTED)
            indices.emplace_back(std::move(buffer_indices));
        else if (validation != Validator::Result::TOO_SMALL)
            indices_rejected.emplace_back(std::move(buffer_indices));
    }

private:
    MaskType offsets;
    int      cluster_count;

    std::vector<EntryStatistical*>   &entries;
    std::vector<pcl::PointIndices> &indices;
    std::vector<pcl::PointIndices> &indices_rejected;
    StructureType                  &array;
    DataIndex                       min_index;
    DataIndex                       max_index;

    Validator                       validator;
    pcl::PointIndices               buffer_indices;
    math::Distribution<3>           buffer_distribution;

    inline void clusterEntry(EntryStatistical *entry)
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

            EntryStatistical *neighbour = array.at(array_index);
            if(!neighbour)
                continue;
            if(neighbour->cluster > -1)
                continue;
            assert(neighbour->cluster == -1);

            if (validator.params.cluster_distance_and_weights[0] != 0.0)
            {
                using MeanType = math::Distribution<3>::PointType;
                MeanType diff = entry->distribution.getMean() - neighbour->distribution.getMean();
                diff(0) *= validator.params.cluster_distance_and_weights[1];
                diff(1) *= validator.params.cluster_distance_and_weights[2];
                diff(2) *= validator.params.cluster_distance_and_weights[3];
                auto dist = diff.dot(diff);
                if (dist > validator.params.cluster_distance_and_weights[0])
                    continue;
            }

            const int cluster = entry->cluster;
            neighbour->cluster = cluster;

            buffer_distribution += neighbour->distribution;
            buffer_indices.indices.insert(buffer_indices.indices.end(),
                                          neighbour->indices.begin(),
                                          neighbour->indices.end());

            clusterEntry(neighbour);
        }
    }
};


}


#endif // FILTERED_CLUSTERING_HPP
