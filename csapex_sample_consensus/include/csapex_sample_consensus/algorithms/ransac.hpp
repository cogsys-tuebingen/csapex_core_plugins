#ifndef RANSAC_HPP
#define RANSAC_HPP

/// PROJECT
#include "sac.hpp"

/// SYSTEM
#include <random>
#include <set>

namespace csapex_sample_consensus {
struct RansacParameters : public Parameters {
    double      inlier_start_probability = 0.99;
    int         random_seed = -1;
    std::size_t maximum_sampling_retries = 100;

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
            std::random_device rd;
            rng_ = std::default_random_engine(rd());
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
        const double log_probability = std::log(1.0 - parameters_.inlier_start_probability);
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
        std::size_t iteration = 0;
        double mean_distance = std::numeric_limits<double>::max();
        while(iteration < k && skipped < maximum_skipped) {
            if(iteration > parameters_.maximum_iterations)
                break;

            if(!selectSamples(model, model_dimension, model_samples)) {
                break;
            }

            for(const int i : model_samples) {
                std::cout << i << " ";
            }
            std::cout << std::endl;


            if(!model->computeModelCoefficients(model_samples)) {
                ++skipped;
                continue;
            }

            typename SampleConsensusModel<PointT>::InlierStatistic stat;
            model->getInlierStatistic(Base::indices_, parameters_.model_search_distance, stat);
            if(stat.count > maximum_inliers) {
                maximum_inliers = stat.count;
                mean_distance = stat.mean_distance;
                best_model = model->clone();

                double w = maximum_inliers * one_over_indices;
                double p_no_outliers = 1.0 - std::pow(w, static_cast<double>(model_dimension));
                p_no_outliers = std::max(std::numeric_limits<double>::epsilon (), p_no_outliers);
                p_no_outliers = std::min(1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);
                k = log_probability / std::log(p_no_outliers);
            }
            ++iteration;
        }

        std::swap(model, best_model);
        return model.get() != nullptr;
    }

protected:
    RansacParameters                           parameters_;
    std::default_random_engine                 rng_;
    std::uniform_int_distribution<std::size_t> distribution_;

    inline bool selectSamples(const typename Model::Ptr &model,
                              const std::size_t          samples,
                              std::vector<int>          &indices)
    {
        std::set<int> selection;
        std::size_t iteration = 0;
        do {
            if(iteration >= parameters_.maximum_sampling_retries)
                return false;

            indices.clear();
            do {
                int next = distribution_(rng_);
                selection.insert(next);
            } while(selection.size() < samples);
            for(const int i : selection) {
                indices.emplace_back(i);
            }
        } while(!model->validateSamples(indices));
        return true;
    }
};
}

#endif // RANSAC_HPP
