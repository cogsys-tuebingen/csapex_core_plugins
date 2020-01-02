#ifndef SAC_MODEL_NORMAL_PLANE_H
#define SAC_MODEL_NORMAL_PLANE_H

/// PROJECT
#include "sac_model_from_normals.h"

namespace csapex_sample_consensus
{
namespace models
{
template <typename PointT, typename NormalT>
class NormalPlane : public ModelFromNormals<PointT, NormalT>
{
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using NormalCloud = pcl::PointCloud<NormalT>;
    using Base = ModelFromNormals<PointT, NormalT>;

    NormalPlane(const typename PointCloud::ConstPtr& pointcloud, const typename NormalCloud::ConstPtr& normalcloud, const float normal_distance_weight = 0.f);

    typename Base::Ptr clone() const override;

    bool isValid() const override;

    bool optimizeModelCoefficients(const float maximum_distance) override;

    bool optimizeModelCoefficients(const std::vector<int>& src_indices, const float maximum_distance) override;

    bool validateSamples(const std::vector<int>& indices) const override;

    bool validateSamples(const std::set<int>& indices) const override;

    std::size_t getModelDimension() const override;

    double getDistanceToModel(const int& index) const override;

    void getDistancesToModel(const std::vector<int>& indices, std::vector<float>& distances) const override;

protected:
    bool doComputeModelCoefficients(const std::vector<int>& indices) override;
    inline float dot(const PointT& p) const;
};
}  // namespace models
}  // namespace csapex_sample_consensus

#endif  // SAC_MODEL_NORMAL_PLANE_H
