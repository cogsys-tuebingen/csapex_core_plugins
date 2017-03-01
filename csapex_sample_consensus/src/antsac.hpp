#ifndef ANTSAC_HPP
#define ANTSAC_HPP

/// PROJECT
#include "sac.hpp"

/// SYSTEM
#include <random>
#include <set>

namespace sample_consensus {

template <typename PointT>
class Antsac : public SampleConsensus<PointT>
{
public:
    using Ptr   = std::shared_ptr<Antsac>;
    using Base  = SampleConsensus<PointT>;
    using Model = typename Base::Model;

    struct Parameters : SampleConsensus<PointT>::Parameters {
        int         random_seed = -1;
        std::size_t maximum_sampling_iterations = 100;

        double      rho = 0.9;
        double      alpha = 0.1;
        double      theta = 0.025;

        Parameters() = default;
    };


    Antsac(const std::vector<int> &indices,
           const Parameters &parameters) :
        Base(indices),
        parameters_(parameters),
        distribution_(0.0, 1.0),
        mean_inliers_(0.0),
        one_over_indices_(1.0 / static_cast<double>(Base::indices_.size())),
        tau_(Base::indices_.size(), one_over_indices_),
        distances_(Base::indices_.size(), std::numeric_limits<double>::max()),
        U_(Base::indices_.size())
    {
        if(parameters_.random_seed >= 0) {
            rng_ = std::default_random_engine(parameters_.random_seed);
        } else {
            rng_ = std::default_random_engine(std::random_device());
        }
    }

    Antsac(const std::size_t cloud_size,
           const Parameters &parameters):
        Base(cloud_size),
        parameters_(parameters),
        distribution_(0.0, 1.0),
        mean_inliers_(0.0),
        one_over_indices_(1.0 / static_cast<double>(Base::indices_.size())),
        tau_(Base::indices_.size(), one_over_indices_),
        distances_(Base::indices_.size(), std::numeric_limits<double>::max()),
        U_(Base::indices_.size())
    {
        if(parameters_.random_seed >= 0) {
            rng_ = std::default_random_engine(parameters_.random_seed);
        } else {
            rng_ = std::default_random_engine(std::random_device());
        }
    }

    virtual void setIndices(const std::vector<int> &indices) override
    {
        Base::setIndices(indices);
        mean_inliers_ = 0.0;
        const std::size_t indices_size = Base::indices_.size();
        one_over_indices_ = 1.0 / static_cast<double>(indices_size);
        tau_.resize(indices_size, one_over_indices_);
        distances_.resize(indices_size, std::numeric_limits<double>::max());
        U_.resize(indices_size);
    }

    virtual bool computeModel(typename SampleConsensusModel<PointT>::Ptr &model) override
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
            mean_inliers_ = (iteration * mean_inliers_ + stat.count) / (iteration + 1.0);

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
            updateTau(stat.count);
            ++iteration;
        }

        std::swap(model, best_model);
        return model.get() != nullptr;
    }

protected:
    Parameters                                 parameters_;
    std::default_random_engine                 rng_;
    std::uniform_real_distribution<double>     distribution_;

    double              mean_inliers_;
    double              one_over_indices_;
    std::vector<double> tau_;
    std::vector<double> distances_;
    std::vector<double> U_;

    inline void updateU()
    {
        const std::size_t indices_size = Base::indices_.size();
        U_.back() = one_over_indices_;
        for(int k = (int) indices_size - 2 ; k >= 0 ; --k) {
            double u_ = std::pow(distribution_(rng_), one_over_indices_);
            U_[k] = U_[k+1] * u_;
        }
    }

    inline void updateTau(const std::size_t inliers_size)
    {
        const std::size_t indices_size = Base::indices_.size();
        double delta_tau = inliers_size / (indices_size + mean_inliers_);
        double sum_tau = 0.0;
        for(std::size_t i = 0 ; i < indices_size ; ++i) {
            tau_[i] = parameters_.rho * tau_[i] + delta_tau * std::exp(-0.5 * (distances_[i] / parameters_.theta));
            sum_tau += tau_[i];
        }
        for(double &t : tau_) {
            t /= sum_tau;
        }
    }

    inline bool selectSamples(const typename Model::Ptr &model,
                              const std::size_t          samples,
                              std::vector<int>          &indices)
    {
        indices.clear();

        auto drawTuple = [&indices, samples, this]() {
            std::set<int> tuple;

            double cumsum_last = 0.0;
            double cumsum      = tau_.front();
            auto in_range = [&cumsum, &cumsum_last](double u) {
                return u >= cumsum_last && u < cumsum;
            };

            std::size_t index = 0;
            for(auto u : U_) {
                while(!in_range(u)) {
                    ++index;
                    cumsum_last = cumsum;
                    cumsum += tau_[index];
                }
                tuple.insert(Base::indices_[index]);
                if(tuple.size() >= samples)
                    break;
            }

            indices.assign(tuple.begin(), tuple.end());
        };

        for(std::size_t i = 0 ; i < parameters_.maximum_sampling_iterations ; ++i) {
            updateU();
            drawTuple();
            if(model->validateSamples(indices))
                return true;
        }
        return false;
    }


};

}

#endif // ANTSAC_HPP
