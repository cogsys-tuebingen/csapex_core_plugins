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

    SampleConsensusModel(const typename PointCloud::ConstPtr &pointcloud) :
        pointcloud_(pointcloud)
    {
    }

    virtual SampleConsensusModel::Ptr clone() const = 0;

    bool computeModelCoefficients(const std::vector<int> &indices)
    {
        if(doComputeModelCoefficients(indices)) {
            model_indices_ = indices;
            return true;
        }
        return false;
    }

    bool getModelCoefficients(Coefficients &coefficients) const
    {
        coefficients = model_coefficients_;
        return isModelValid();
    }

    bool getModelIndices(std::vector<int> &indices) const
    {
        indices = model_indices_;
        return isModelValid();
    }

    virtual bool        isModelValid() const = 0;
    virtual bool        validateSamples(const std::vector<int> &indices) const = 0;
    virtual std::size_t getModelDimension() const = 0;
    virtual double      getDistanceToModel(const int &index) const = 0;
    virtual void        getDistancesToModel(const std::vector<int> &indices, std::vector<float> &distances) const = 0;

    std::size_t countInliers(const std::vector<int> &indices,
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

    void getInliers(const float maximum_distance,
                    std::vector<int> &dst_indices) const
    {
        const std::size_t size = pointcloud_->size();
        dst_indices.reserve(size);
        for(std::size_t i = 0 ; i < size ; ++i) {
            if(getDistanceToModel(i) <= maximum_distance) {
                dst_indices.emplace_back(i);
            }
        }
    }

    void getInliers(const std::vector<int> &src_indices,
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

    void getOutliers(const std::vector<int> &src_indices,
                     const float maximum_distance,
                     std::vector<int> &dst_indices)
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

    Coefficients getModelCoefficients() const
    {
        return model_coefficients_;
    }

    std::vector<int> getModelSampleInices()
    {
        return model_indices_;
    }

protected:
    virtual bool   doComputeModelCoefficients(const std::vector<int> &indices) = 0;

    typename PointCloud::ConstPtr pointcloud_;
    Coefficients                  model_coefficients_;
    std::vector<int>              model_indices_;

};
}

#endif // MODEL_HPP
