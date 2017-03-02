#ifndef SAC_HPP
#define SAC_HPP

#include <memory>
#include <csapex_sample_consensus/models/sac_model.hpp>

namespace csapex_sample_consensus {
template<typename PointT>
class SampleConsensus  {
public:
    using Ptr = std::shared_ptr<SampleConsensus>;
    using Model = SampleConsensusModel<PointT>;

    struct Parameters {
        enum TerminationCriteria {MAX_ITERATION = 1,    /// maximum iteration reached
                                  MIN_DISTANCE  = 2,    /// minimum mean distance reached
                                  MAX_RETRY     = 4     /// maximum retries to find better model
                                 };

        inline bool terminate(const std::size_t iteration,
                              const float mean_distance,
                              const std::size_t retries,
                              const std::size_t inliers) const
        {
            bool t = false;
            t |= (MAX_ITERATION & termination_criteria) && iteration >= maximum_iterations;
            t |= (MIN_DISTANCE & termination_criteria) && mean_distance <= maximum_mean_distance;
            t |= (MAX_RETRY & termination_criteria) && retries >= maximum_retries;
            return t && (inliers >= minimum_inliers);
        }

        float            maximum_model_distance = 0.1;
        double           probability            = 0.99;

        int termination_criteria = (MAX_ITERATION | MIN_DISTANCE);

        float            maximum_mean_distance  = 0.05;
        std::size_t      maximum_iterations     = 5000;
        std::size_t      maximum_retries        = 500;
        std::size_t      minimum_inliers        = 10000;


    };

    SampleConsensus(const std::vector<int> &indices) :
        indices_(indices)
    {
    }

    SampleConsensus(const std::size_t cloud_size)
    {
        indices_.resize(cloud_size);
        int index = -1;
        for(int &i : indices_) {
            i = ++index;
        }
    }

    virtual bool computeModel(typename Model::Ptr &model) = 0;

    virtual void setIndices(const std::vector<int> &indices)
    {
        indices_ = indices;
    }

protected:
    std::vector<int> indices_;

};
}

#endif // SAC_HPP
