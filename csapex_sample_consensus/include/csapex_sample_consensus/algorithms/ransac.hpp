#ifndef RANSAC_HPP
#define RANSAC_HPP

/// PROJECT
#include "sac.hpp"

/// SYSTEM
#include <random>
#include <set>

namespace csapex_sample_consensus {
struct RansacParameters : public Parameters {
    int         random_seed = -1;
    std::size_t maximum_sampling_iterations = 100;

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
           const RansacParameters &parameters) :
        Base(indices),
        parameters_(parameters),
        distribution_(0, Base::indices_.size() - 1)
    {
        if(parameters_.random_seed >= 0) {
            rng_ = std::default_random_engine(parameters_.random_seed);
        } else {
            rng_ = std::default_random_engine(std::random_device());
        }
    }

    Ransac(const std::size_t cloud_size,
           const RansacParameters &parameters) :
        Base(cloud_size),
        parameters_(parameters),
        distribution_(0, Base::indices_.size() - 1)
    {
        if(parameters_.random_seed >= 0) {
            rng_ = std::default_random_engine(parameters_.random_seed);
        } else {
            std::random_device rd;
            rng_ = std::default_random_engine(rd());
        }
    }

    virtual void setIndices(const std::vector<int> &indices) override
    {
        Base::setIndices(indices);
        distribution_ = std::uniform_int_distribution<std::size_t>(0, Base::indices_.size() - 1);
    }

    virtual bool computeModel(typename Model::Ptr &model) override
    {
        const double log_probability = std::log(1.0 - parameters_.probability);
        const double one_over_indices = 1.0 / static_cast<double>(Base::indices_.size());
        const std::size_t model_dimension = model->getModelDimension();
        const std::size_t maximum_skipped = parameters_.maximum_iterations * 10;

        if(Base::indices_.size() < model_dimension)
            return false;

        int maximum_inliers = 0;
        typename Model::Ptr best_model;

        double k = 1.0;
        std::size_t skipped = 0;

        std::vector<int> model_samples;
        std::size_t retries = 0;
        std::size_t iteration = 0;
        double mean_distance = 0.0;
        while(!parameters_.terminate(iteration, mean_distance, retries, maximum_inliers)) {
            if(iteration >= k || skipped >= maximum_skipped)
                break;

            if(!selectSamples(model, model_dimension, model_samples)) {
                break;
            }

            if(!model->computeModelCoefficients(model_samples)) {
                ++skipped;
                continue;
            }

            typename SampleConsensusModel<PointT>::InlierStatistic stat;
            model->getInlierStatistic(Base::indices_, parameters_.maximum_model_distance, stat);
            if(stat.count > maximum_inliers) {
                maximum_inliers = stat.count;
                mean_distance = stat.mean_distance;
                best_model = model->clone();

                double w = maximum_inliers * one_over_indices;
                double p_no_outliers = std::min(std::max(1.0 - std::pow(w, static_cast<double>(model_dimension)),
                                                         std::numeric_limits<double>::epsilon()),
                                                1.0 - std::numeric_limits<double>::epsilon());
                k = log_probability / std::log(p_no_outliers);
            } else {
                ++retries;
            }
            ++iteration;
        }

        std::swap(model, best_model);
        return model.get() != nullptr;
    }

protected:
    RansacParameters                                 parameters_;
    std::default_random_engine                 rng_;
    std::uniform_int_distribution<std::size_t> distribution_;

    inline bool selectSamples(const typename Model::Ptr &model,
                              const std::size_t          samples,
                              std::vector<int> &indices)
    {
        indices.clear();

        auto drawTuple = [&indices, samples, this]() {
              std::set<int> tuple;
              while(tuple.size() < samples) {
                tuple.insert(distribution_(rng_));
              }
              indices.assign(tuple.begin(), tuple.end());
        };

        for(std::size_t i = 0 ; i < parameters_.maximum_sampling_iterations ; ++i) {
            drawTuple();

            if(model->validateSamples(indices))
                return true;
        }
        return false;
    }
};
}

#endif // RANSAC_HPP
