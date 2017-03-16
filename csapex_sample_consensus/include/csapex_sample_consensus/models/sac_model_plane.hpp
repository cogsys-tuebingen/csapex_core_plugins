#ifndef SAC_MODEL_PLANE_HPP
#define SAC_MODEL_PLANE_HPP

/// PROJECT
#include "sac_model.hpp"

/// SYSTEM
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

namespace csapex_sample_consensus {
template<typename PointT>
class ModelPlane : public SampleConsensusModel<PointT> {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using Base = SampleConsensusModel<PointT>;

    ModelPlane(const typename PointCloud::ConstPtr &pointcloud):
       Base(pointcloud)
    {
    }

    virtual typename Base::Ptr clone() const override
    {
        ModelPlane *plane = new ModelPlane(Base::pointcloud_);
        plane->model_coefficients_ = Base::model_coefficients_;
        plane->model_indices_ = Base::model_indices_;
        return typename Base::Ptr(plane);
    }

    virtual bool isModelValid() const override
    {
        return Base::model_coefficients_.size() == 4;
    }

    virtual bool optimizeModelCoefficients(const float maximum_distance) override
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

    virtual bool optimizeModelCoefficients(const std::vector<int> &src_indices,
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

    virtual bool validateSamples(const std::vector<int> &indices) const override
    {
        if(indices.size() != 3)
            return false;

        const PointT p0 = Base::pointcloud_->at(indices[0]);
        const PointT p1 = Base::pointcloud_->at(indices[1]);
        const PointT p2 = Base::pointcloud_->at(indices[2]);

        if(isNan(p0) ||
                isNan(p1) ||
                    isNan(p2)) {
            return false;
        }


        const Eigen::Vector3f dy1dy2 = {(p1.x - p0.x) / (p2.x - p0.x),
                                        (p1.y - p0.y) / (p2.y - p0.y),
                                        (p1.z - p0.z) / (p2.z - p0.z)};

        return ( (dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]) );
    }

    virtual bool validateSamples(const std::set<int> &indices) const override
    {
        if(indices.size() != 3)
            return false;

        auto it = indices.begin();
        const PointT p0 = Base::pointcloud_->at(*(it));
        ++it;
        const PointT p1 = Base::pointcloud_->at(*(it));
        ++it;
        const PointT p2 = Base::pointcloud_->at(*(it));

        if(isNan(p0) ||
                isNan(p1) ||
                    isNan(p2)) {
            return false;
        }

        const Eigen::Vector3f dy1dy2 = {(p1.x - p0.x) / (p2.x - p0.x),
                                        (p1.y - p0.y) / (p2.y - p0.y),
                                        (p1.z - p0.z) / (p2.z - p0.z)};

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

        return dot(Base::pointcloud_->at(index));
    }

    virtual void getDistancesToModel(const std::vector<int> &indices,
                                     std::vector<float> &distances) const override
    {
        if(!isModelValid())
            return;

        const std::size_t size = indices.size();
        distances.resize(size, std::numeric_limits<float>::lowest());
        for(std::size_t i = 0 ; i < size ; ++i) {
            distances[i] = dot(Base::pointcloud_->at(indices[i]));
        }
    }

protected:
    virtual bool doComputeModelCoefficients(const std::vector<int> &indices) override
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

        auto dot = [](const PointT &p,
                      const typename Base::Coefficients &c)
        {
            return std::abs(p.x * c[0] +
                            p.y * c[1] +
                            p.z * c[2] +
                            c[3]);
        };

        Base::model_coefficients_.normalize ();
        float d = -1 * dot(p0, Base::model_coefficients_);
        Base::model_coefficients_[3] = d;

        return true;
    }

    inline float dot(const PointT &p) const
    {
        return std::abs(p.x * Base::model_coefficients_[0] +
                        p.y * Base::model_coefficients_[1] +
                        p.z * Base::model_coefficients_[2] +
                        Base::model_coefficients_[3]);
    }

    inline bool isNan(const PointT &p) const
    {
        return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
    }

};
}

#endif // SAC_MODEL_PLANE_HPP
