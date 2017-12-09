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
    bool        use_outlier_probability  = false;
    int         maximum_sampling_retries = 100;

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
        const std::size_t indices_size = Base::indices_.size();
        const std::size_t validation_samples = indices_size * parameters_.model_validation_ratio;
        if(indices_size < model_dimension)
            return false;


        InternalParameters internal_params(parameters_, model_dimension);

        /// SETUP THE TERMINATION CRITERIA
        delegate<bool()> termination  = [&internal_params, this](){
            bool terminate_max_skipped          = internal_params.skipped >= internal_params.maximum_skipped;
            bool terminate_max_iteration        = (int) internal_params.iteration >= parameters_.maximum_iterations;
            bool terminate_outlier_probability  = parameters_.use_outlier_probability &&
                                                  internal_params.iteration >= internal_params.k_outlier;
            bool terminate_mean_model_distance  = parameters_.use_mean_model_distance &&
                                                  internal_params.mean_model_distance < parameters_.mean_model_distance;
            return terminate_max_skipped || terminate_max_iteration || terminate_outlier_probability || terminate_mean_model_distance;
        };

        delegate<void()> update_internal_paramters = [&internal_params, this]() {
            if(parameters_.use_outlier_probability) {
                double maximum_inlier_ratio = internal_params.maximum_inliers * one_over_indices_;
                double p_no_outliers = 1.0 - std::pow(maximum_inlier_ratio, static_cast<double>(internal_params.model_dimension));
                p_no_outliers = std::max(std::numeric_limits<double>::epsilon (), p_no_outliers);
                p_no_outliers = std::min(1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);
                internal_params.k_outlier = internal_params.log_outlier_probability / std::log(p_no_outliers);
            }
        };


        /// ITERATE AND FIND A MODEL
        while(!termination()) {
            if(!selectSamples(model, internal_params.model_dimension, internal_params.model_samples)) {
                break;
            }

            if(!model->computeModelCoefficients(internal_params.model_samples)) {
                ++internal_params.skipped;
                continue;
            }

            typename Model::InlierStatistic stat;
            if(parameters_.model_validation_ratio == 0.0 ||
                    parameters_.model_validation_ratio == 1.0) {
                model->getInlierStatistic(Base::indices_, parameters_.model_search_distance, stat);
            } else {
                std::vector<int> indices;
                drawSamples(validation_samples, indices);
                model->getInlierStatistic(indices, parameters_.model_search_distance, stat);
            }

            if(stat.count > internal_params.maximum_inliers) {
                internal_params.maximum_inliers = stat.count;
                internal_params.best_model = model->clone();
                internal_params.mean_model_distance = stat.mean_distance;
                update_internal_paramters();
            }
            ++internal_params.iteration;
        }

        std::swap(model, internal_params.best_model);
        return model.get() != nullptr;
    }

protected:
    struct InternalParameters {
        const double        log_outlier_probability;
        const std::size_t   maximum_skipped;
        const std::size_t   model_dimension;

        double              k_outlier = 1.0;
        std::size_t         skipped   = 0;
        std::size_t         iteration = 0;

        std::size_t         maximum_inliers = 0;
        typename Model::Ptr best_model;

        std::vector<int>    model_samples;

        double              mean_model_distance = std::numeric_limits<double>::max();

        InternalParameters(const RansacParameters &params,
                           const std::size_t model_dimension) :
            log_outlier_probability(std::log(1.0 - params.outlier_probability)),
            maximum_skipped(params.maximum_iterations * 10),
            model_dimension(model_dimension)
        {
        }
    };

    RansacParameters                           parameters_;
    std::default_random_engine                 &rng_;
    std::uniform_int_distribution<std::size_t> distribution_;
    double                                     one_over_indices_;

    inline bool selectSamples(const typename Model::Ptr &model,
                              const std::size_t          samples,
                              std::vector<int>          &indices)
    {
        std::set<int> selection;
        int iteration = 0;
        bool valid = false;

        while(!valid && iteration < parameters_.maximum_sampling_retries) {
            selection.clear();
            while(selection.size() < samples) {
                int next = Base::indices_[distribution_(rng_)];
                selection.insert(next);
            }
            valid = model->validateSamples(selection);
            ++iteration;
        }

        indices.clear();
        for(const int i : selection) {
            indices.emplace_back(i);
        }
        return valid;
    }

    inline void drawSamples(const std::size_t samples,
                            std::vector<int> &indices)
    {
        std::size_t size = 0;
        std::set<int> selection;
        for(std::size_t i = 0 ; i < samples ; ++i) {
            const int next = Base::indices_[distribution_(rng_)];
            selection.insert(next);
            const std::size_t selection_size = selection.size();
            if(selection_size > size) {
                indices.emplace_back(next);
                size = selection_size;
            }
        }
    }
};
}

#endif // RANSAC_HPP
