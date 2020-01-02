#ifndef SAC_MODEL_H
#define SAC_MODEL_H

/// SYSTEM
#include <memory>
#include <pcl/point_cloud.h>
#include <set>

namespace csapex_sample_consensus
{
namespace models
{
template <typename PointT>
class Model
{
public:
    using Ptr = std::shared_ptr<Model>;
    using PointCloud = pcl::PointCloud<PointT>;
    using Coefficients = Eigen::VectorXf;

    struct InlierStatistic
    {
        double mean_distance = -1.0;
        std::size_t count = -1;
    };

    Model(const typename PointCloud::ConstPtr& pointcloud);
    
    virtual ~Model() = default;

    inline bool computeModelCoefficients(const std::vector<int>& indices);

    inline bool getModelCoefficients(Coefficients& coefficients) const;

    inline bool getModelIndices(std::vector<int>& indices) const;

    inline std::size_t countInliers(const std::vector<int>& indices, const float maximum_distance) const;

    inline void getInlierStatistic(const std::vector<int>& indices, const float maximum_distance, InlierStatistic& statistic) const;

    inline void getInlierStatistic(const std::vector<int>& indices, const float maximum_distance, InlierStatistic& statistic, std::vector<double>& distances) const;

    inline void getInliers(const float maximum_distance, std::vector<int>& dst_indices) const;

    inline void getInliers(const std::vector<int>& src_indices, const float maximum_distance, std::vector<int>& dst_indices) const;

    inline void getOutliers(const float maximum_distance, std::vector<int>& dst_indices) const;

    inline void getOutliers(const std::vector<int>& src_indices, const float maximum_distance, std::vector<int>& dst_indices) const;

    inline void getInliersAndOutliers(const std::vector<int>& src_indices, const float maximum_distance, std::vector<int>& dst_inliers, std::vector<int>& dst_outliers) const;

    inline void getInliersAndOutliers(const float maximum_distance, std::vector<int>& dst_inliers, std::vector<int>& dst_outliers) const;

    /// Methods to be implemented by the different model types
    virtual Model::Ptr clone() const = 0;
    virtual bool isValid() const = 0;
    virtual bool optimizeModelCoefficients(const float inlier_distance) = 0;
    virtual bool optimizeModelCoefficients(const std::vector<int>& src_indices, const float inlier_distance) = 0;
    virtual bool validateSamples(const std::set<int>& indices) const = 0;
    virtual bool validateSamples(const std::vector<int>& indices) const = 0;
    virtual std::size_t getModelDimension() const = 0;
    virtual double getDistanceToModel(const int& index) const = 0;
    virtual void getDistancesToModel(const std::vector<int>& indices, std::vector<float>& distances) const = 0;

protected:
    typename PointCloud::ConstPtr pointcloud_;
    Coefficients model_coefficients_;
    std::vector<int> model_indices_;

    virtual bool doComputeModelCoefficients(const std::vector<int>& indices) = 0;
    inline bool isNan(const PointT& p) const;
};
}  // namespace models
}  // namespace csapex_sample_consensus

#endif  // SAC_MODEL_H
