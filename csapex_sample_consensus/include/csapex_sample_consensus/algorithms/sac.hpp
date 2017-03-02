#ifndef SAC_HPP
#define SAC_HPP

#include <memory>
#include <csapex_sample_consensus/models/sac_model.hpp>

namespace csapex_sample_consensus {
struct Parameters {
    enum TerminationCriteria {MAX_ITERATION = 1,    /// maximum iteration reached
                              MIN_DISTANCE  = 2,    /// minimum mean distance reached
                              MAX_RETRY     = 4     /// maximum retries to find better model
                             };

    inline bool terminate(const std::size_t iteration,
                          const double mean_distance,
                          const std::size_t retries,
                          const double inliner_percentage) const
    {
        bool t = false;
        t |= (MAX_ITERATION & termination_criteria) && iteration >= maximum_iterations;
        t |= (MIN_DISTANCE & termination_criteria) && mean_distance <= maximum_mean_model_distance;
        t |= (MAX_RETRY & termination_criteria) && retries >= maximum_retries;
        return t && (inliner_percentage >= minimum_inlier_percentage);
    }

    int termination_criteria = (MAX_ITERATION | MIN_DISTANCE);

    double           model_search_distance        = 0.1;    /// samples in this distance are considered for fitting
    double           maximum_mean_model_distance  = 0.05;   /// the maximum allowed mean distance
    std::size_t      maximum_iterations           = 5000;   /// mean distance to the model
    std::size_t      maximum_retries              = 500;    /// times of retries in case of not finding any model
    double           minimum_inlier_percentage    = .5f;    /// the mimium amount of inliers

};

template<typename PointT>
class SampleConsensus  {
public:
    using Ptr = std::shared_ptr<SampleConsensus>;
    using Model = SampleConsensusModel<PointT>;

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

    inline const std::vector<int> & getIndices() const
    {
        return indices_;
    }

protected:
    std::vector<int> indices_;

};
}

#endif // SAC_HPP
