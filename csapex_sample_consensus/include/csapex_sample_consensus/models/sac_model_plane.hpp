#ifndef SAC_MODEL_PLANE_HPP
#define SAC_MODEL_PLANE_HPP

/// PROJECT
#include "sac_model_plane.h"

namespace csapex_sample_consensus {
namespace models {
template<typename PointT>
Plane<PointT>::Plane(const typename PointCloud::ConstPtr &pointcloud) :
    Base(pointcloud)
{
}

template<typename PointT>
inline typename Plane<PointT>::Base::Ptr Plane<PointT>::clone() const
{
    Plane *plane = new Plane(Base::pointcloud_);
    plane->model_coefficients_ = Base::model_coefficients_;
    plane->model_indices_ = Base::model_indices_;
    return typename Base::Ptr(plane);
}



template<typename PointT>
inline bool Plane<PointT>::isModelValid() const
{
    return Base::model_coefficients_.size() == 4;
}

template<typename PointT>
inline bool Plane<PointT>::optimizeModelCoefficients(const float maximum_distance)
{
    std::vector<int> indices;
    Base::getInliers(maximum_distance, indices);

    if(indices.size() == 0)
        return false;

    Eigen::Matrix3d cov;
    Eigen::Vector4d centroid;
    pcl::computeMeanAndCovarianceMatrix(*Base::pointcloud_, indices, cov, centroid);
    Eigen::Vector3d eigen_vector;
    double          eigen_value;
    pcl::eigen33(cov, eigen_value, eigen_vector);

    Base::model_coefficients_[0] = eigen_vector[0];
    Base::model_coefficients_[1] = eigen_vector[1];
    Base::model_coefficients_[2] = eigen_vector[2];
    Base::model_coefficients_[3] = 0;
    Base::model_coefficients_[3] = -1 * Base::model_coefficients_.dot (centroid.cast<float>());
    return true;
}

template<typename PointT>
inline bool Plane<PointT>::optimizeModelCoefficients(const std::vector<int> &src_indices,
                                                          const float maximum_distance)
{
    std::vector<int> indices;
    Base::getInliers(src_indices, maximum_distance, indices);

    if(indices.size() == 0)
        return false;

    Eigen::Matrix3d cov;
    Eigen::Vector4d centroid;
    pcl::computeMeanAndCovarianceMatrix(*Base::pointcloud_, indices, cov, centroid);
    Eigen::Vector3d eigen_vector;
    double          eigen_value;
    pcl::eigen33(cov, eigen_value, eigen_vector);

    Base::model_coefficients_[0] = eigen_vector[0];
    Base::model_coefficients_[1] = eigen_vector[1];
    Base::model_coefficients_[2] = eigen_vector[2];
    Base::model_coefficients_[3] = 0;
    Base::model_coefficients_[3] = -1 * Base::model_coefficients_.dot (centroid.cast<float>());
    return true;
}

template<typename PointT>
inline bool Plane<PointT>::validateSamples(const std::vector<int> &indices) const
{
    if(indices.size() != 3)
        return false;

    const PointT p0 = Base::pointcloud_->at(indices[0]);
    const PointT p1 = Base::pointcloud_->at(indices[1]);
    const PointT p2 = Base::pointcloud_->at(indices[2]);

    if(Base::isNan(p0) ||
            Base::isNan(p1) ||
                Base::isNan(p2)) {
        return false;
    }


    const Eigen::Vector3f dy1dy2 = {(p1.x - p0.x) / (p2.x - p0.x),
                                    (p1.y - p0.y) / (p2.y - p0.y),
                                    (p1.z - p0.z) / (p2.z - p0.z)};

    return ( (dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]) );
}

template<typename PointT>
inline bool Plane<PointT>::validateSamples(const std::set<int> &indices) const
{
    if(indices.size() != 3)
        return false;

    auto it = indices.begin();
    const PointT p0 = Base::pointcloud_->at(*(it));
    ++it;
    const PointT p1 = Base::pointcloud_->at(*(it));
    ++it;
    const PointT p2 = Base::pointcloud_->at(*(it));

    if(Base::isNan(p0) ||
            Base::isNan(p1) ||
                Base::isNan(p2)) {
        return false;
    }

    const Eigen::Vector3f dy1dy2 = {(p1.x - p0.x) / (p2.x - p0.x),
                                    (p1.y - p0.y) / (p2.y - p0.y),
                                    (p1.z - p0.z) / (p2.z - p0.z)};

    return ( (dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]) );
}

template<typename PointT>
inline std::size_t Plane<PointT>::getModelDimension() const
{
    return 3ul;
}

template<typename PointT>
inline double Plane<PointT>::getDistanceToModel(const int &index) const
{
    if(!isModelValid())
        return std::numeric_limits<float>::lowest();

    return dot(Base::pointcloud_->at(index));
}

template<typename PointT>
inline void Plane<PointT>::getDistancesToModel(const std::vector<int> &indices,
                                                    std::vector<float> &distances) const
{
    if(!isModelValid())
        return;

    const std::size_t size = indices.size();
    distances.resize(size, std::numeric_limits<float>::lowest());
    for(std::size_t i = 0 ; i < size ; ++i) {
        distances[i] = dot(Base::pointcloud_->at(indices[i]));
    }
}


template<typename PointT>
inline bool Plane<PointT>::doComputeModelCoefficients(const std::vector<int> &indices)
{
    if(indices.size() != 3) {
        return false;
    }

    const PointT &p0 = Base::pointcloud_->at(indices[0]);
    const PointT &p1 = Base::pointcloud_->at(indices[1]);
    const PointT &p2 = Base::pointcloud_->at(indices[2]);
    const Eigen::Vector4f p1p0 = {p1.x - p0.x, p1.y - p0.y, p1.z - p0.z, 0.f};
    const Eigen::Vector4f p2p0 = {p2.x - p0.x, p2.y - p0.y, p2.z - p0.z, 0.f};

    /// Check for collinearity
    const Eigen::Vector4f dy1dy2 = p1p0.cwiseQuotient(p2p0);
    if ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) ) {
        return false;
    }

    Base::model_coefficients_.resize(4);
    Base::model_coefficients_[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
    Base::model_coefficients_[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
    Base::model_coefficients_[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];
    Base::model_coefficients_[3] = 0;

    Base::model_coefficients_.normalize ();
    float d = -1 * dot(p0);
    Base::model_coefficients_[3] = d;

    return true;
}
template<typename PointT>
inline float Plane<PointT>::dot(const PointT &p) const
{
    return std::abs(p.x * Base::model_coefficients_[0] +
            p.y * Base::model_coefficients_[1] +
            p.z * Base::model_coefficients_[2] +
            Base::model_coefficients_[3]);
}
}
}

#endif // SAC_MODEL_PLANE_HPP
