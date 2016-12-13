#ifndef FILTERED_CLUSTERING_HPP
#define FILTERED_CLUSTERING_HPP

#include <cslibs_kdtree/fill.hpp>
#include <cslibs_kdtree/index.hpp>

#include "entry.hpp"
#include "validator.hpp"
#include "color_differences.hpp"

namespace csapex {
using AO                  = kdtree::ArrayOperations<3, int, int>;
using AOA                 = kdtree::ArrayOperations<3, int, std::size_t>;

template<typename StructureType>
class FilteredClusteringMono
{
public:
    typedef kdtree::detail::fill<DataIndex, 3>   MaskFiller;
    typedef typename MaskFiller::Type            MaskType;
    typedef typename StructureType::Index        StructureIndex;
    typedef EntryStatisticalMono                 EntryType;
    typedef Validator<ClusterParamsStatisticalMono> ValidatorType;

    FilteredClusteringMono(const ClusterParamsStatisticalMono &_params,
                           std::vector<pcl::PointIndices>      &_indices,
                           std::vector<pcl::PointIndices>      &_indices_rejected,
                           StructureType                       &_array,
                           DataIndex                           &_min_index,
                           DataIndex                           &_max_index) :
        cluster_count(0),
        indices(_indices),
        indices_rejected(_indices_rejected),
        array(_array),
        min_index(_min_index),
        max_index(_max_index),
        validator(_params, buffer_indices, buffer_distribution)
    {
        MaskFiller::assign(offsets);
    }

    template<typename Itr>
    inline void cluster(Itr begin, Itr end)
    {
        for(; begin != end; ++begin)
        {
            auto& entry = *(*begin);
            if (!entry.valid)
                continue;
            if(entry.cluster > -1)
                continue;

            buffer_distribution.reset();

            entry.cluster = cluster_count;
            ++cluster_count;
            clusterEntry(entry);

            ValidatorType::Result validation = validator.validate();
            if (validation == ValidatorType::Result::ACCEPTED) {
                indices.emplace_back(std::move(buffer_indices));
                distributions.emplace_back(std::move(validator.getBufferDistribution()));
            }
            else
            {
                if (validation != ValidatorType::Result::TOO_SMALL) {
                    indices_rejected.emplace_back(std::move(buffer_indices));
                } else {
                    buffer_indices.indices.clear();
                }
            }
        }

        /// cluster vertically consecutive
        /// track cluster borders and direction of border
        /// if mainly vertical borders exist, it can be considered fusing vertically
    }

private:
    MaskType offsets;
    int      cluster_count;

    std::vector<pcl::PointIndices>      &indices;
    std::vector<pcl::PointIndices>      &indices_rejected;
    std::vector<math::Distribution<3>>  distributions;

    StructureType                  &array;
    DataIndex                       min_index;
    DataIndex                       max_index;

    ValidatorType                   validator;              /// TODO : refactor that
    pcl::PointIndices               buffer_indices;
    math::Distribution<3>           buffer_distribution;


    inline bool isVertical(const DataIndex &offset)
    {
        return offset[0] == 0 && offset[1] == 0 && offset[2] != 0;
    }

    inline void clusterEntry(EntryType& entry)
    {
        StructureIndex array_index;
        DataIndex index;
        for(DataIndex &offset : offsets) {
            if(AO::is_zero(offset))
                continue;

            AO::add(entry.index, offset, index);

            bool out_of_bounds = false;
            for(std::size_t j = 0 ; j < 3 ; ++j) {
                out_of_bounds |= index[j] < min_index[j];
                out_of_bounds |= index[j] > max_index[j];
                array_index[j] = index[j] - min_index[j];
            }

            if(out_of_bounds)
                continue;

            EntryType& neighbour = array.at(array_index);
            if(!neighbour.valid)
                continue;
            if(neighbour.cluster > -1)
                continue;

            if (validator.params.cluster_distance_and_weights[0] != 0.0)
            {
                using MeanType = math::Distribution<3>::PointType;
                MeanType diff = entry.distribution.getMean() - neighbour.distribution.getMean();
                diff(0) *= validator.params.cluster_distance_and_weights[1];
                diff(1) *= validator.params.cluster_distance_and_weights[2];
                diff(2) *= validator.params.cluster_distance_and_weights[3];
                auto dist = diff.dot(diff);
                if (dist > validator.params.cluster_distance_and_weights[0])
                    continue;
            }
            if(validator.params.mono_difference) {
                double diff = entry.mono_mean.getMean() - neighbour.mono_mean.getMean();
                if(diff > validator.params.mono_difference) {
                    continue;
                }
            }

            const int cluster = entry.cluster;
            neighbour.cluster = cluster;

            buffer_distribution += neighbour.distribution;
            buffer_indices.indices.insert(buffer_indices.indices.end(),
                                          neighbour.indices.begin(),
                                          neighbour.indices.end());

            clusterEntry(neighbour);
        }
    }
};


}


#endif // FILTERED_CLUSTERING_HPP
