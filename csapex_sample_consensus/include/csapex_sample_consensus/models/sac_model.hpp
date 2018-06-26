#ifndef MODEL_HPP
#define MODEL_HPP

/// PROJECT
#include "sac_model.h"

namespace csapex_sample_consensus {
namespace models {
template<typename PointT>
Model<PointT>::Model(const typename PointCloud::ConstPtr &pointcloud) :
    pointcloud_(pointcloud)
{
}

template<typename PointT>
inline bool Model<PointT>::computeModelCoefficients(const std::vector<int> &indices)
{
    if(doComputeModelCoefficients(indices)) {
        model_indices_ = indices;
        return true;
    }
    return false;
}

template<typename PointT>
inline bool Model<PointT>::getModelCoefficients(Coefficients &coefficients) const
{
    coefficients = model_coefficients_;
    return isValid();
}

template<typename PointT>
inline bool Model<PointT>::getModelIndices(std::vector<int> &indices) const
{
    indices = model_indices_;
    return isValid();
}



template<typename PointT>
inline std::size_t Model<PointT>::countInliers(const std::vector<int> &indices,
                                                              const float maximum_distance) const
{
    if(!isValid())
        return 0;

    std::size_t count = 0;
    for(const int i : indices) {
        if(getDistanceToModel(i) <= maximum_distance) {
            ++count;
        }
    }
    return count;
}



template<typename PointT>
inline void Model<PointT>::getInlierStatistic(const std::vector<int> &indices,
                                                             const float maximum_distance,
                                                             InlierStatistic &statistic) const
{
    if(!isValid())
        return;

    statistic.count = 0;
    statistic.mean_distance = 0;

    double distance = 0.0;
    for(const int i : indices) {
        distance = getDistanceToModel(i);
        if(distance <= maximum_distance) {
            statistic.mean_distance += distance;
            ++statistic.count;
        }
    }
    if(statistic.count != 0)
        statistic.mean_distance /= static_cast<double>(statistic.count);
}

template<typename PointT>
inline void Model<PointT>::getInlierStatistic(const std::vector<int> &indices,
                                                             const float maximum_distance,
                                                             InlierStatistic &statistic,
                                                             std::vector<double> &distances) const
{
    if(!isValid())
        return;

    statistic.count = 0;
    statistic.mean_distance = 0;

    distances.clear();

    double distance = 0.0;
    for(const int i : indices) {
        distance = getDistanceToModel(i);
        distances.emplace_back(distance);
        if(distance <= maximum_distance) {
            statistic.mean_distance += distance;
            ++statistic.count;
        }
    }
    if(statistic.count != 0)
        statistic.mean_distance /= static_cast<double>(statistic.count);
}


template<typename PointT>
inline void Model<PointT>::getInliers(const float maximum_distance,
                                                     std::vector<int> &dst_indices) const
{
    if(!isValid())
        return;

    const std::size_t size = pointcloud_->size();
    dst_indices.reserve(size);
    for(std::size_t i = 0 ; i < size ; ++i) {
        if(getDistanceToModel(i) <= maximum_distance) {
            dst_indices.emplace_back(i);
        }
    }
}

template<typename PointT>
inline void Model<PointT>::getInliers(const std::vector<int> &src_indices,
                                                     const float maximum_distance,
                                                     std::vector<int> &dst_indices) const
{
    if(!isValid())
        return;

    dst_indices.reserve(src_indices.size());
    for(const int i : src_indices) {
        if(getDistanceToModel(i) <= maximum_distance) {
            dst_indices.emplace_back(i);
        }
    }
}

template<typename PointT>
inline void Model<PointT>::getOutliers(const float maximum_distance,
                                                      std::vector<int> &dst_indices) const
{
    if(!isValid())
        return;

    const std::size_t size = pointcloud_->size();
    dst_indices.reserve(size);
    for(std::size_t i = 0 ; i < size ; ++i) {
        if(getDistanceToModel(i) > maximum_distance) {
            dst_indices.emplace_back(i);
        }
    }
}

template<typename PointT>
inline void Model<PointT>::getOutliers(const std::vector<int> &src_indices,
                                       const float maximum_distance,
                                       std::vector<int> &dst_indices) const
{
    if(!isValid())
        return;

    dst_indices.reserve(src_indices.size());
    for(const int i : src_indices) {
        if(getDistanceToModel(i) > maximum_distance) {
            dst_indices.emplace_back(i);
        }
    }
}

template<typename PointT>
inline void Model<PointT>::getInliersAndOutliers(const std::vector<int> &src_indices,
                                                 const float maximum_distance,
                                                 std::vector<int> &dst_inliers,
                                                 std::vector<int> &dst_outliers) const
{
    if(!isValid())
        return;

    dst_inliers.reserve(src_indices.size());
    dst_outliers.reserve(dst_outliers.size());
    for(const int i : src_indices) {
        if(getDistanceToModel(i) <= maximum_distance) {
            dst_inliers.emplace_back(i);
        } else {
            dst_outliers.emplace_back(i);
        }
    }
}


template<typename PointT>
inline void Model<PointT>::getInliersAndOutliers(const float maximum_distance,
                                                 std::vector<int> &dst_inliers,
                                                 std::vector<int> &dst_outliers) const
{
    if(!isValid())
        return;

    const std::size_t size = pointcloud_->size();
    dst_inliers.reserve(size);
    dst_outliers.reserve(size);
    for(std::size_t i = 0 ; i < size ; ++i) {
        if(getDistanceToModel(i) <= maximum_distance) {
            dst_inliers.emplace_back(i);
        } else {
            dst_outliers.emplace_back(i);
        }
    }
}

template<typename PointT>
inline bool Model<PointT>::isNan(const PointT &p) const
{
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}
}
}

#endif // MODEL_HPP
