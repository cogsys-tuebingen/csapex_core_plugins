#ifndef SAC_SEGMENTATION_HPP
#define SAC_SEGMENTATION_HPP

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

namespace csapex {

const static int SAC_ANTSAC = 7;

template<typename PointT>
class SACSegmentation : public pcl::SACSegmentation<PointT>
{
protected:
    using Base = pcl::SACSegmentation<PointT>;

    void initSAC (const int method_type) override
    {
        if (Base::sac_)
            Base::sac_.reset ();
        // Build the sample consensus method
        if(method_type == SAC_ANTSAC) {
            // Set the Sample Consensus parameters if they are given/changed
            if (Base::sac_->getProbability () != Base::probability_)
            {
                PCL_DEBUG ("[pcl::%s::initSAC] Setting the desired probability to %f\n", Base::getClassName ().c_str (), Base::probability_);
                Base::sac_->setProbability (Base::probability_);
            }
            if (Base::max_iterations_ != -1 && Base::sac_->getMaxIterations () != Base::max_iterations_)
            {
                PCL_DEBUG ("[pcl::%s::initSAC] Setting the maximum number of iterations to %d\n", Base::getClassName ().c_str (), Base::max_iterations_);
                Base::sac_->setMaxIterations (Base::max_iterations_);
            }
            if (Base::samples_radius_ > 0.)
            {
                PCL_DEBUG ("[pcl::%s::initSAC] Setting the maximum sample radius to %f\n", Base::getClassName ().c_str (), Base::samples_radius_);
                // Set maximum distance for radius search during random sampling
                Base::model_->setSamplesMaxDist (Base::samples_radius_, Base::samples_radius_search_);
            }
        } else {
            Base::initSAC(method_type);
        }
    }
};

template<typename PointT, typename NormalT>
class SACSegmentationFromNormals : public pcl::SACSegmentationFromNormals<PointT, NormalT>
{
protected:
    using Base = pcl::SACSegmentationFromNormals<PointT, NormalT>;
    void initSAC (const int method_type) override
    {
        if (Base::sac_)
            Base::sac_.reset ();
        // Build the sample consensus method
        if(method_type == SAC_ANTSAC) {
            // Set the Sample Consensus parameters if they are given/changed
            if (Base::sac_->getProbability () != Base::probability_)
            {
                PCL_DEBUG ("[pcl::%s::initSAC] Setting the desired probability to %f\n", Base::getClassName ().c_str (), Base::probability_);
                Base::sac_->setProbability (Base::probability_);
            }
            if (Base::max_iterations_ != -1 && Base::sac_->getMaxIterations () != Base::max_iterations_)
            {
                PCL_DEBUG ("[pcl::%s::initSAC] Setting the maximum number of iterations to %d\n", Base::getClassName ().c_str (), Base::max_iterations_);
                Base::sac_->setMaxIterations (Base::max_iterations_);
            }
            if (Base::samples_radius_ > 0.)
            {
                PCL_DEBUG ("[pcl::%s::initSAC] Setting the maximum sample radius to %f\n", Base::getClassName ().c_str (), Base::samples_radius_);
                // Set maximum distance for radius search during random sampling
                auto model_ptr = Base::getModel();
                model_ptr->setSamplesMaxDist (Base::samples_radius_, Base::samples_radius_search_);
            }
        } else {
            Base::initSAC(method_type);
        }
    }
};
}



#endif // SAC_SEGMENTATION_HPP
