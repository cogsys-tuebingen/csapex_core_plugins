#ifndef MODEL_HPP
#define MODEL_HPP

#include <memory>

#include <pcl/point_cloud.h>

namespace sample_consensus {
template<typename PointT>
class SampleConsensusModel {
public:
    using Ptr          = std::shared_ptr<SampleConsensusModel>;
    using PointCloud   = pcl::PointCloud<PointT>;
    using Coefficients = Eigen::VectorXf;

    struct InlierStatistic {
        double      mean_distance = -1.0;
        std::size_t count = -1;
    };

    SampleConsensusModel(const typename PointCloud::ConstPtr &pointcloud) :
        pointcloud_(pointcloud)
    {
    }

    inline bool computeModelCoefficients(const std::vector<int> &indices)
    {
        if(doComputeModelCoefficients(indices)) {
            model_indices_ = indices;
            return true;
        }
        return false;
    }

    inline bool getModelCoefficients(Coefficients &coefficients) const
    {
        coefficients = model_coefficients_;
        return isModelValid();
    }

    inline bool getModelIndices(std::vector<int> &indices) const
    {
        indices = model_indices_;
        return isModelValid();
    }

    inline std::size_t countInliers(const std::vector<int> &indices,
                                    const float maximum_distance) const
    {
        if(!isModelValid())
            return 0;

        std::size_t count = 0;
        for(const int i : indices) {
            if(getDistanceToModel(i) <= maximum_distance) {
                ++count;
            }
        }
        return count;
    }

    inline void getInlierStatistic(const std::vector<int> &indices,
                                   const float maximum_distance,
                                   InlierStatistic &statistic) const
    {
        if(!isModelValid())
            return;

        statistic.count = 0;
        statistic.mean_distance = 0.0;

        double distance = 0.0;
        for(const int i : indices) {
            distance = getDistanceToModel(i);
            if(distance <= maximum_distance) {
                statistic.mean_distance += distance;
                ++statistic.count;
            }
        }
        statistic.mean_distance /= static_cast<double>(statistic.count);
    }

    inline void getInliers(const float maximum_distance,
                           std::vector<int> &dst_indices) const
    {
        if(!isModelValid())
            return;

        const std::size_t size = pointcloud_->size();
        dst_indices.reserve(size);
        for(std::size_t i = 0 ; i < size ; ++i) {
            if(getDistanceToModel(i) <= maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }
    }

    inline void getInliers(const std::vector<int> &src_indices,
                           const float maximum_distance,
                           std::vector<int> &dst_indices) const
    {
        if(!isModelValid())
            return;

        dst_indices.reserve(src_indices.size());
        for(const int i : src_indices) {
            if(getDistanceToModel(i) <= maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }
    }

    inline void getOutliers(const float maximum_distance,
                           std::vector<int> &dst_indices) const
    {
        if(!isModelValid())
            return;

        const std::size_t size = pointcloud_->size();
        dst_indices.reserve(size);
        for(std::size_t i = 0 ; i < size ; ++i) {
            if(getDistanceToModel(i) > maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }
    }

    inline void getOutliers(const std::vector<int> &src_indices,
                            const float maximum_distance,
                            std::vector<int> &dst_indices) const
    {
        if(!isModelValid())
            return;

        dst_indices.reserve(src_indices.size());
        for(const int i : src_indices) {
            if(getDistanceToModel(i) > maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }
    }

    inline void getInliersAndOutliers(const std::vector<int> &src_indices,
                                     const float maximum_distance,
                                     std::vector<int> &dst_inliers,
                                     std::vector<int> &dst_outliers) const
    {
        if(!isModelValid())
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


    inline void getInliersAndOutliers(const float maximum_distance,
                                     std::vector<int> &dst_inliers,
                                     std::vector<int> &dst_outliers) const
    {
        if(!isModelValid())
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


    /// Methods to be implemented by the different model types
    virtual SampleConsensusModel::Ptr clone() const = 0;
    virtual bool                      isModelValid() const = 0;
    virtual bool                      validateSamples(const std::vector<int> &indices) const = 0;
    virtual std::size_t               getModelDimension() const = 0;
    virtual double                    getDistanceToModel(const int &index) const = 0;
    virtual void                      getDistancesToModel(const std::vector<int> &indices, std::vector<float> &distances) const = 0;

protected:
    virtual bool   doComputeModelCoefficients(const std::vector<int> &indices) = 0;

    typename PointCloud::ConstPtr pointcloud_;
    Coefficients                  model_coefficients_;
    std::vector<int>              model_indices_;

};
}

#endif // MODEL_HPP
