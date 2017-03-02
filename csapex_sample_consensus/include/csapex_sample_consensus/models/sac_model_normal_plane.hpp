#ifndef SAC_MODEL_NORMAL_PLANE_HPP
#define SAC_MODEL_NORMAL_PLANE_HPP

/// PROJECT
#include "sac_model_from_normals.hpp"

/// SYSTEM
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace csapex_sample_consensus {
template<typename PointT, typename NormalT>
class ModelNormalPlane : public SampleConsensusModelFromNormals<PointT, NormalT> {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using Base = SampleConsensusModel<PointT>;

    ModelNormalPlane(const typename PointCloud::ConstPtr &pointcloud):
       Base(pointcloud)
    {
    }

    virtual typename Base::Ptr clone() const override
    {
        ModelNormalPlane *plane = new ModelNormalPlane(Base::pointcloud_, Base::normalcloud_);
        plane->model_coefficients_ = Base::model_coefficients_;
        plane->model_indices_ = Base::model_indices_;
        return typename Base::Ptr(plane);
    }

    virtual bool isModelValid() const override
    {
        return Base::model_coefficients_.size() == 4;
    }

    virtual bool validateSamples(const std::vector<int> &indices) const override
    {
        if(indices.size() != 3)
            return false;

        Eigen::Vector4f p0 = convert(Base::pointcloud_->at(indices[0]));
        Eigen::Vector4f p1 = convert(Base::pointcloud_->at(indices[1]));
        Eigen::Vector4f p2 = convert(Base::pointcloud_->at(indices[2]));
        Eigen::Vector4f dy1dy2 = (p1-p0).cwiseQuotient(p2-p0);

        return ( (dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]) );
    }

    virtual std::size_t getModelDimension() const override
    {
        return 3;
    }

    virtual double getDistanceToModel(const int &index) const override
    {
        if(!isModelValid())
            return std::numeric_limits<float>::lowest();

        Eigen::Vector4f coefficients = Base::model_coefficients_;
        coefficients[3] = 0.f;

        const auto &nt = Base::normalcloud_->at(index);

        Eigen::Vector4f p = convert(Base::pointcloud_->at(index));
        Eigen::Vector4f n = convert(nt);
        float eucledian = std::abs(coefficients.dot(p) + Base::model_coefficients_[3]);
        float angular   = std::abs(pcl::getAngle3D(n, coefficients));
        angular = std::min(angular, static_cast<float>(M_PI - angular));

        float weight = Base::normal_distance_weight_ * (1.f - nt.curvature);
        return std::abs(weight * angular + (1.f - weight) * eucledian);

    }

    virtual void getDistancesToModel(const std::vector<int> &indices,
                                     std::vector<float> &distances) const override
    {
        if(!isModelValid())
            return;

        Eigen::Vector4f coefficients = Base::model_coefficients_;
        coefficients[3] = 0.f;

        const std::size_t size = indices.size();
        distances.resize(size, std::numeric_limits<float>::lowest());
        for(std::size_t i = 0 ; i < size ; ++i) {
            const int index = indices[i];
            const auto &nt = Base::normalcloud_->at(index);

            Eigen::Vector4f p = convert(Base::pointcloud_->at(index));
            Eigen::Vector4f n = convert(nt);
            float eucledian = std::abs(coefficients.dot(p) + Base::model_coefficients_[3]);
            float angular   = std::abs(pcl::getAngle3D(n, coefficients));
            angular = std::min(angular, static_cast<float>(M_PI - angular));

            float weight = Base::normal_distance_weight_ * (1.f - nt.curvature);
            distances[i] = std::abs(weight * angular + (1.f - weight) * eucledian);
        }
    }

protected:
    virtual bool doComputeModelCoefficients(const std::vector<int> &indices) override
    {
        if(indices.size() != 3) {
            return false;
        }

        Eigen::Vector4f p0 = convert(Base::pointcloud_->at(indices[0]));
        Eigen::Vector4f p1 = convert(Base::pointcloud_->at(indices[1]));
        Eigen::Vector4f p2 = convert(Base::pointcloud_->at(indices[2]));
        Eigen::Vector4f p1p0 = p1 - p0;
        Eigen::Vector4f p2p0 = p2 - p0;

        /// Check for collinearity
        Eigen::Vector4f dy1dy2 = p1p0.cwiseQuotient(p2p0);
        if ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) ) {
            return false;
        }

        Base::model_coefficients_.resize(4);
        Base::model_coefficients_[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
        Base::model_coefficients_[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
        Base::model_coefficients_[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];
        Base::model_coefficients_[3] = 0;

        Base::model_coefficients_.normalize ();
        Base::model_coefficients_[3] = -1 * (Base::model_coefficients_.template head<4>().dot (p0.matrix ()));

        return true;
    }

    inline Eigen::Vector4f convert(const PointT &pt, const float w = 0.f) const
    {
        return Eigen::Vector4f(pt.x, pt.y, pt.z, w);
    }

    inline Eigen::Vector4f convert(const NormalT &n, const float w = 0.f) const
    {
        return Eigen::Vector4f(n.x, n.y, n.z, w);
    }

};
}

#endif // SAC_MODEL_NORMAL_PLANE_HPP
