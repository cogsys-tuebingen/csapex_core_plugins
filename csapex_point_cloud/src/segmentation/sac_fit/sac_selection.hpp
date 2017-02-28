#ifndef SAC_SELECTION_HPP
#define SAC_SELECTION_HPP

#include "sac_model.hpp"

namespace sample_consensus {
template<typename PointT>
class SACSelection {
public:
    using Ptr = std::shared_ptr<SACSelection>;
    using PointCloud   = pcl::PointCloud<PointT>;

    SACSelection(const typename PointCloud::ConstPtr &pointcloud)
    {
    }

    void select(const SampleConsensusModel<PointT>::Ptr &model,
                const float maximum_distance,
                std::vector<int> &dst_indices) const
    {
        const auto &m = *model;
        const std::size_t size = pointcloud_->size();
        for(std::size_t i = 0 ; i < size ; ++i) {
            const auto &p  = pointcloud_->at(i);
            if(m.distanceToModel(p) <= maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }
    }

    void selectInverse(const SampleConsensusModel<PointT>::Ptr &model,
                       const float maximum_distance,
                       std::vector<int> &dst_indices) const
    {
        const auto &m = *model;
        const std::size_t size = pointcloud_->size();
        for(std::size_t i = 0 ; i < size ; ++i) {
            const auto &p  = pointcloud_->at(i);
            if(m.distanceToModel(p) > maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }
    }

    void select(const SampleConsensusModel<PointT>::Ptr &model,
                const float maximum_distance,
                const std::vector<int> &src_indices,
                std::vector<int>       &dst_indices) const
    {
        const auto &m = *model;
        for(const int i : src_indices) {
            const auto &p  = pointcloud_->at(i);
            if(m.distanceToModel(p) <= maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }

    }

    void selectInverse(const SampleConsensusModel<PointT>::Ptr &model,
                       const float maximum_distance,
                       const std::vector<int> &src_indices,
                       std::vector<int> &dst_indices) const
    {
        const auto &m = *model;
        for(const int i : src_indices) {
            const auto &p  = pointcloud_->at(i);
            if(m.distanceToModel(p) > maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }
    }

private:
    PointCloud::ConstPtr pointcloud_;

};
}

#endif // SAC_SELECTION_HPP
