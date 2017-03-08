#ifndef SAC_MODEL_PLANE_HPP
#define SAC_MODEL_PLANE_HPP

/// PROJECT
#include "sac_model.hpp"

/// SYSTEM
#include <pcl/point_types.h>

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

    virtual bool validateSamples(const std::vector<int> &indices) const override
    {
        if(indices.size() != 3)
            return false;

        Eigen::Vector4f p0 = convert(Base::pointcloud_->at(indices[0]));
        Eigen::Vector4f p1 = convert(Base::pointcloud_->at(indices[1]));
        Eigen::Vector4f p2 = convert(Base::pointcloud_->at(indices[2]));

        if(isNan(p0) ||
                isNan(p1) ||
                    isNan(p2)) {
            return false;
        }


        Eigen::Vector4f dy1dy2 = (p1-p0).cwiseQuotient(p2-p0);

        return ( (dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]) );
    }

    virtual bool validateSamples(const std::set<int> &indices) const override
    {
        if(indices.size() != 3)
            return false;

        auto it = indices.begin();
        Eigen::Vector4f p0 = convert(Base::pointcloud_->at(*(it)));
        ++it;
        Eigen::Vector4f p1 = convert(Base::pointcloud_->at(*(it)));
        ++it;
        Eigen::Vector4f p2 = convert(Base::pointcloud_->at(*(it)));

        if(isNan(p0) ||
                isNan(p1) ||
                    isNan(p2)) {
            return false;
        }

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
        Eigen::Vector4f pt = convert(Base::pointcloud_->at(index), 1.f);
        return std::abs(Base::model_coefficients_.dot(pt));
    }

    virtual void getDistancesToModel(const std::vector<int> &indices,
                                     std::vector<float> &distances) const override
    {
        if(!isModelValid())
            return;

        const std::size_t size = indices.size();
        distances.resize(size, std::numeric_limits<float>::lowest());
        for(std::size_t i = 0 ; i < size ; ++i) {
            distances[i] = std::abs(Base::model_coefficients_.dot(convert(Base::pointcloud_->at(indices[i]), 1.f)));
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
        float d = -1 * Base::model_coefficients_.dot(p0);
        Base::model_coefficients_[3] = d;

        return true;
    }

    inline Eigen::Vector4f convert(const PointT &pt, const float w = 0.f) const
    {
        return Eigen::Vector4f(pt.x, pt.y, pt.z, w);
    }

    inline bool isNan(const Eigen::Vector4f &v) const
    {
        return std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2]) || std::isnan(v[3]);
    }
};
}

#endif // SAC_MODEL_PLANE_HPP
