#ifndef SAC_MODEL_NORMAL_PLANE_HPP
#define SAC_MODEL_NORMAL_PLANE_HPP

/// PROJECT
#include "sac_model_normal_plane.h"
#include <csapex_point_cloud/math/plane.hpp>

/// SYSTEM
#include <pcl/common/common.h>
#include <pcl/point_types.h>

namespace csapex_sample_consensus
{
namespace models
{
template <typename PointT, typename NormalT>
NormalPlane<PointT, NormalT>::NormalPlane(const typename PointCloud::ConstPtr& pointcloud, const typename NormalCloud::ConstPtr& normalcloud, const float normal_distance_weight)
  : Base(pointcloud, normalcloud, normal_distance_weight)
{
}

template <typename PointT, typename NormalT>
inline typename NormalPlane<PointT, NormalT>::Base::Ptr NormalPlane<PointT, NormalT>::clone() const
{
    NormalPlane<PointT, NormalT>* plane = new NormalPlane<PointT, NormalT>(Base::pointcloud_, Base::normalcloud_, Base::normal_distance_weight_);
    plane->model_coefficients_ = Base::model_coefficients_;
    plane->model_indices_ = Base::model_indices_;
    return typename Base::Ptr(plane);
}

template <typename PointT, typename NormalT>
inline bool NormalPlane<PointT, NormalT>::isValid() const
{
    return Base::model_coefficients_.size() == 4;
}

template <typename PointT, typename NormalT>
inline bool NormalPlane<PointT, NormalT>::optimizeModelCoefficients(const float maximum_distance)
{
    std::vector<int> indices;
    Base::getInliers(maximum_distance, indices);

    if (indices.size() == 0)
        return false;

    csapex::math::Distribution<3> distribution;
    for (const int i : indices) {
        const auto& p = Base::pointcloud_->at(i);
        distribution.add({ p.x, p.y, p.z });
    }

    Eigen::Vector3d x_0, normal;
    if (!csapex::math::Plane::fit(distribution, x_0, normal))
        return false;

    Base::model_coefficients_[0] = normal(0);
    Base::model_coefficients_[1] = normal(1);
    Base::model_coefficients_[2] = normal(2);
    Base::model_coefficients_[3] = 0;
    Base::model_coefficients_[3] = -1 * normal.dot(x_0);
    return true;
}

template <typename PointT, typename NormalT>
inline bool NormalPlane<PointT, NormalT>::optimizeModelCoefficients(const std::vector<int>& src_indices, const float maximum_distance)
{
    std::vector<int> indices;
    Base::getInliers(src_indices, maximum_distance, indices);

    if (indices.size() == 0)
        return false;

    csapex::math::Distribution<3> distribution;
    for (const int i : indices) {
        const auto& p = Base::pointcloud_->at(i);
        distribution.add({ p.x, p.y, p.z });
    }

    Eigen::Vector3d x_0, normal;
    if (!csapex::math::Plane::fit(distribution, x_0, normal))
        return false;

    Base::model_coefficients_[0] = normal(0);
    Base::model_coefficients_[1] = normal(1);
    Base::model_coefficients_[2] = normal(2);
    Base::model_coefficients_[3] = 0;
    Base::model_coefficients_[3] = -1 * normal.dot(x_0);
    return true;
}

template <typename PointT, typename NormalT>
inline bool NormalPlane<PointT, NormalT>::validateSamples(const std::vector<int>& indices) const
{
    if (indices.size() != 3)
        return false;

    const PointT p0 = Base::pointcloud_->at(indices[0]);
    const PointT p1 = Base::pointcloud_->at(indices[1]);
    const PointT p2 = Base::pointcloud_->at(indices[2]);

    if (Base::isNan(p0) || Base::isNan(p1) || Base::isNan(p2)) {
        return false;
    }

    const Eigen::Vector3f dy1dy2 = { (p1.x - p0.x) / (p2.x - p0.x), (p1.y - p0.y) / (p2.y - p0.y), (p1.z - p0.z) / (p2.z - p0.z) };

    return ((dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]));
}

template <typename PointT, typename NormalT>
inline bool NormalPlane<PointT, NormalT>::validateSamples(const std::set<int>& indices) const
{
    if (indices.size() != 3)
        return false;

    auto it = indices.begin();
    const PointT p0 = Base::pointcloud_->at(*(it));
    ++it;
    const PointT p1 = Base::pointcloud_->at(*(it));
    ++it;
    const PointT p2 = Base::pointcloud_->at(*(it));

    if (Base::isNan(p0) || Base::isNan(p1) || Base::isNan(p2)) {
        return false;
    }

    const Eigen::Vector3f dy1dy2 = { (p1.x - p0.x) / (p2.x - p0.x), (p1.y - p0.y) / (p2.y - p0.y), (p1.z - p0.z) / (p2.z - p0.z) };

    return ((dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]));
}

template <typename PointT, typename NormalT>
inline std::size_t NormalPlane<PointT, NormalT>::getModelDimension() const
{
    return 3ul;
}

template <typename PointT, typename NormalT>
inline double NormalPlane<PointT, NormalT>::getDistanceToModel(const int& index) const
{
    if (!isValid())
        return std::numeric_limits<float>::lowest();

    Eigen::Vector4f coefficients = Base::model_coefficients_;
    coefficients[3] = 0.f;

    const NormalT& nt = Base::normalcloud_->at(index);
    const PointT& p = Base::pointcloud_->at(index);

    Eigen::Vector4f n = { nt.normal_x, nt.normal_y, nt.normal_z, 0.f };
    float eucledian = std::abs(dot(p) + Base::model_coefficients_[3]);
    float angular = std::abs(pcl::getAngle3D(n, coefficients));
    angular = std::min(angular, static_cast<float>(M_PI - angular));

    float weight = Base::normal_distance_weight_ * (1.f - nt.curvature);
    return std::abs(weight * angular + (1.f - weight) * eucledian);
}

template <typename PointT, typename NormalT>
inline void NormalPlane<PointT, NormalT>::getDistancesToModel(const std::vector<int>& indices, std::vector<float>& distances) const
{
    if (!isValid())
        return;

    Eigen::Vector4f coefficients = Base::model_coefficients_;
    coefficients[3] = 0.f;

    const std::size_t size = indices.size();
    distances.resize(size, std::numeric_limits<float>::lowest());
    for (std::size_t i = 0; i < size; ++i) {
        const int index = indices[i];
        const NormalT& nt = Base::normalcloud_->at(index);
        const PointT& p = Base::pointcloud_->at(index);

        const Eigen::Vector4f n = { nt.normal_x, nt.normal_y, nt.normal_z, 0.f };
        float eucledian = std::abs(dot(p) + Base::model_coefficients_[3]);
        float angular = std::abs(pcl::getAngle3D(n, coefficients));
        angular = std::min(angular, static_cast<float>(M_PI - angular));

        float weight = Base::normal_distance_weight_ * (1.f - nt.curvature);
        distances[i] = std::abs(weight * angular + (1.f - weight) * eucledian);
    }
}

template <typename PointT, typename NormalT>
inline bool NormalPlane<PointT, NormalT>::doComputeModelCoefficients(const std::vector<int>& indices)
{
    if (indices.size() != 3) {
        return false;
    }

    const PointT& p0 = Base::pointcloud_->at(indices[0]);
    const PointT& p1 = Base::pointcloud_->at(indices[1]);
    const PointT& p2 = Base::pointcloud_->at(indices[2]);
    const Eigen::Vector4f p1p0 = { p1.x - p0.x, p1.y - p0.y, p1.z - p0.z, 0.f };
    const Eigen::Vector4f p2p0 = { p2.x - p0.x, p2.y - p0.y, p2.z - p0.z, 0.f };

    /// Check for collinearity
    const Eigen::Vector4f dy1dy2 = p1p0.cwiseQuotient(p2p0);
    if ((dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1])) {
        return false;
    }

    Base::model_coefficients_.resize(4);
    Base::model_coefficients_[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
    Base::model_coefficients_[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
    Base::model_coefficients_[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];
    Base::model_coefficients_[3] = 0;

    Base::model_coefficients_.normalize();
    float d = -1 * dot(p0);
    Base::model_coefficients_[3] = d;

    return true;
}

template <typename PointT, typename NormalT>
inline float NormalPlane<PointT, NormalT>::dot(const PointT& p) const
{
    return std::abs(p.x * Base::model_coefficients_[0] + p.y * Base::model_coefficients_[1] + p.z * Base::model_coefficients_[2] + Base::model_coefficients_[3]);
}
}  // namespace models
}  // namespace csapex_sample_consensus

#endif  // SAC_MODEL_NORMAL_PLANE_HPP
