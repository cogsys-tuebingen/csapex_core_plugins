#ifndef RANSAC_HPP
#define RANSAC_HPP

/// PROJECT
#include "sac.hpp"
#include "delegate.hpp"

/// SYSTEM
#include <random>
#include <set>

namespace csapex_sample_consensus {
struct RansacParameters : public Parameters {
    double      outlier_probability      = 0.99;
    int         maximum_sampling_retries = 100;
    bool        use_outlier_probability  = false;

    RansacParameters() = default;
};


template<typename PointT>
class Ransac : public SampleConsensus<PointT>
{
public:
    using Ptr   = std::shared_ptr<Ransac>;
    using Base  = SampleConsensus<PointT>;
    using Model = typename Base::Model;

    Ransac(const std::vector<int> &indices,
           const RansacParameters &parameters,
           std::default_random_engine &rng) :
        Base(indices),
        parameters_(parameters),
        rng_(rng),
        distribution_(0, Base::indices_.size() - 1),
        one_over_indices_(1.0 / static_cast<double>(Base::indices_.size()))
    {
    }

    virtual void setIndices(const std::vector<int> &indices) override
    {
        Base::setIndices(indices);
        distribution_ = std::uniform_int_distribution<std::size_t>(0, Base::indices_.size() - 1);
        one_over_indices_ = 1.0 / static_cast<double>(Base::indices_.size());
    }

    virtual bool computeModel(typename Model::Ptr &model) override
    {

        const std::size_t model_dimension = model->getModelDimension();
        if(Base::indices_.size() < model_dimension)
            return false;


        InternalParameters interal_params(parameters_);

        std::size_t         maximum_inliers = 0;
        typename Model::Ptr best_model;
        std::vector<int>    model_samples;

        /// SETUP THE TERMINATION CRITERIA
        delegate<void()> update_internal_paramters;
        delegate<bool()> termination;
        if(parameters_.use_outlier_probability) {
            termination = [&interal_params, this](){
                bool max_skipped = interal_params.skipped >= interal_params.maximum_skipped;
                bool max_iteration = interal_params.iteration >= parameters_.maximum_iterations;
                bool necessary_iterations = interal_params.iteration >= interal_params.k_outlier;
                return max_skipped || max_iteration || necessary_iterations;
            };
            update_internal_paramters = [&interal_params, model_dimension, &maximum_inliers, this]() {
                double maximum_inlier_ratio = maximum_inliers * one_over_indices_;
                double p_no_outliers = 1.0 - std::pow(maximum_inlier_ratio, static_cast<double>(model_dimension));
                p_no_outliers = std::max(std::numeric_limits<double>::epsilon (), p_no_outliers);
                p_no_outliers = std::min(1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);
                interal_params.k_outlier = interal_params.log_outlier_probability / std::log(p_no_outliers);
            };
        } else {
            termination = [&interal_params, this](){
                bool max_skipped = interal_params.skipped >= interal_params.maximum_skipped;
                bool max_iteration = interal_params.iteration >= parameters_.maximum_iterations;
                return max_skipped || max_iteration;
            };
            update_internal_paramters = [](){return;};
        }

        /// ITERATE AND FIND A MODEL
        while(!termination()) {

            if(!selectSamples(model, model_dimension, model_samples)) {
                break;
            }

            if(!model->computeModelCoefficients(model_samples)) {
                ++interal_params.skipped;
                continue;
            }

            typename SampleConsensusModel<PointT>::InlierStatistic stat;
            model->getInlierStatistic(Base::indices_, parameters_.model_search_distance, stat);
            if(stat.count > maximum_inliers) {
                maximum_inliers = stat.count;
                best_model = model->clone();
                update_internal_paramters();
            }
            ++interal_params.iteration;
        }

        std::swap(model, best_model);
        return model.get() != nullptr;
    }

protected:
    RansacParameters                           parameters_;
    std::default_random_engine                 &rng_;
    std::uniform_int_distribution<std::size_t> distribution_;
    double                                     one_over_indices_;

    struct InternalParameters {
        const double log_outlier_probability;
        const std::size_t maximum_skipped;

        double      k_outlier = 1.0;
        std::size_t skipped   = 0;
        std::size_t iteration = 0;

        InternalParameters(const RansacParameters &params) :
            log_outlier_probability(std::log(1.0 - params.outlier_probability)),
            maximum_skipped(params.maximum_iterations * 10)
        {
        }
    };

    inline bool selectSamples(const typename Model::Ptr &model,
                              const std::size_t          samples,
                              std::vector<int>          &indices)
    {
        std::set<int> selection;
        std::size_t iteration = 0;
        bool valid = false;

        while(!valid && iteration < parameters_.maximum_sampling_retries) {
            selection.clear();
            while(selection.size() < samples) {
                int next = distribution_(rng_);
                selection.insert(next);
            }
            valid = model->validateSamples(selection);
        }

        indices.clear();
        for(const int i : selection) {
            indices.emplace_back(i);
        }
        return valid;
    }





};
}

#endif // RANSAC_HPP
