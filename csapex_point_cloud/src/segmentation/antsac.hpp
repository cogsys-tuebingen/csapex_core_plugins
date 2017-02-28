#ifndef ANTSAC_HPP
#define ANTSAC_HPP

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model.h>

#include <random>

namespace csapex {
template<typename PointT>
struct AntSampleConsensusModel : public pcl::SampleConsensusModel<PointT>
{
    virtual bool
    computeModelCoefficients (const std::vector<int> &samples,
                              Eigen::VectorXf &model_coefficients)
    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
        return false;
    }

    virtual void
    optimizeModelCoefficients (const std::vector<int> &inliers,
                               const Eigen::VectorXf &model_coefficients,
                               Eigen::VectorXf &optimized_coefficients)
    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
    }

    virtual void
    getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                         std::vector<double> &distances)
    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
    }

    virtual void
    selectWithinDistance (const Eigen::VectorXf &model_coefficients,
                          const double threshold,
                          std::vector<int> &inliers)
    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
    }

    virtual int
    countWithinDistance (const Eigen::VectorXf &model_coefficients,
                         const double threshold)

    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
        return -1;
    }

    virtual void
    projectPoints (const std::vector<int> &inliers,
                   const Eigen::VectorXf &model_coefficients,
                   pcl::PointCloud<PointT> &projected_points,
                   bool copy_data_fields = true)

    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
    }

    virtual bool
    doSamplesVerifyModel (const std::set<int> &indices,
                          const Eigen::VectorXf &model_coefficients,
                          const double threshold)
    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
        return false;
    }

    virtual inline bool
    isModelValid (const Eigen::VectorXf &model_coefficients)

    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
        return false;
    }

    virtual inline bool
    isSampleGood(const std::vector<int> &samples) const
    {
        throw std::runtime_error("[AntSampleConsensusModel]: Do not use this method!");
        return false;
    }

    inline bool isSampleGood(typename pcl::SampleConsensusModel<PointT>::Ptr &model,
                             const std::vector<int> &samples) const
    {
        return model->isSampleGood(samples);
    }

};



/** \brief @b RandomSampleConsensus represents an implementation of the RANSAC (RAndom SAmple Consensus) algorithm, as
  * described in: "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and
  * Automated Cartography", Martin A. Fischler and Robert C. Bolles, Comm. Of the ACM 24: 381–395, June 1981.
  * \author Radu B. Rusu
  * \ingroup sample_consensus
  */
template <typename PointT>
class AntSampleConsensus : public pcl::SampleConsensus<PointT>
{
    typedef typename pcl::SampleConsensusModel<PointT>::Ptr SampleConsensusModelPtr;

public:
    typedef boost::shared_ptr<AntSampleConsensus> Ptr;
    typedef boost::shared_ptr<const AntSampleConsensus> ConstPtr;

    using pcl::SampleConsensus<PointT>::max_iterations_;
    using pcl::SampleConsensus<PointT>::threshold_;
    using pcl::SampleConsensus<PointT>::iterations_;
    using pcl::SampleConsensus<PointT>::sac_model_;
    using pcl::SampleConsensus<PointT>::model_;
    using pcl::SampleConsensus<PointT>::model_coefficients_;
    using pcl::SampleConsensus<PointT>::inliers_;
    using pcl::SampleConsensus<PointT>::probability_;

    /** \brief RANSAC (RAndom SAmple Consensus) main constructor
      * \param[in] model a Sample Consensus model
      */
    AntSampleConsensus (const SampleConsensusModelPtr &model)
        : pcl::SampleConsensus<PointT> (model),
          rho_(0.9),
          alpha_(0.1),
          theta_(0.025),
          distribution_(0.0, 1.0)
    {
        // Maximum number of trials before we give up.
        max_iterations_ = 10000;
    }

    /** \brief RANSAC (RAndom SAmple Consensus) main constructor
      * \param[in] model a Sample Consensus model
      * \param[in] threshold distance to model threshold
      * \param[in] rho is the pheromone level evaporation [0.0, 1.0]
      * \param[in] alpha sample importance gain [0.1, 2.0]
      */
    AntSampleConsensus (const SampleConsensusModelPtr &model,
                        double threshold,
                        double rho = 0.9,
                        double alpha = 0.1,
                        double theta = 0.025)
        : pcl::SampleConsensus<PointT> (model, threshold),
          rho_(rho),
          alpha_(alpha),
          theta_(theta),
          distribution_(0.0, 1.0)
    {
        // Maximum number of trials before we give up.
        max_iterations_ = 10000;
    }

    /** \brief Compute the actual model and find the inliers
      * \param[in] debug_verbosity_level enable/disable on-screen debug information and set the verbosity level
      */
    bool
    computeModel (int debug_verbosity_level = 0) override
    {
        // Warn and exit if no threshold was set
        if (threshold_ == std::numeric_limits<double>::max())
        {
            PCL_ERROR ("[pcl::AntSampleConsensus::computeModel] No threshold set!\n");
            return (false);
        }


        iterations_ = 0;
        int n_best_inliers_count = -INT_MAX;
        double k = 1.0;

        Eigen::VectorXf model_coefficients;



        const auto        indices = sac_model_->getIndices();
        const std::size_t indices_size = sac_model_->getIndices()->size();
        const std::size_t model_size = sac_model_->getSampleSize();

        if(indices_size < model_size) {
            PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] Too few points in pointcloud!\n");
            return false;
        }

        const double log_probability  = log (1.0 - probability_);
        const double one_over_indices = 1.0 / static_cast<double> (indices_size);

        int    n_inliers_count = 0;
        double n_inliers_mean = 0.0;
        std::vector<double> tau(indices_size, one_over_indices);
        std::vector<double> distances(indices_size);
        std::vector<double> U;
        std::vector<int> selection(model_size, -1);

        /// update the pheromnone matrix - keep it normalized
        auto updateTau = [&tau, indices_size, n_inliers_mean,  distances, n_inliers_count, this]() {
            double delta_tau = n_inliers_count / (indices_size + n_inliers_mean);
            double sum_tau = 0.0;
            for(std::size_t i = 0 ; i < indices_size ; ++i) {
                tau[i] = rho_ * tau[i] + delta_tau * std::exp(-0.5 * (distances[i] / theta_));
                sum_tau += tau[i];
            }
            for(double &t : tau) {
                t /= sum_tau;
            }
        };

        /// prepare the cumulative sum for each step
        auto updateU = [&U, indices_size, one_over_indices, this] () {
            U.resize(indices_size, one_over_indices);
            for(int k = (int) indices_size - 2 ; k >= 0 ; --k) {
                double u_ = std::pow(distribution_(rne_), one_over_indices);
                U[k] = U[k+1] * u_;
            }
        };
        /// draw samples according to the pheromone matrix
        auto draw = [&tau, &selection, &indices, model_size, U, this] () {
            AntSampleConsensusModel<PointT> ant_model;
            double cumsum_last = 0.0;
            double cumsum      = tau.front();
            int last_drawn = -1;

            assert(model_size == selection.size());

            auto in_range = [&cumsum, &cumsum_last](double u) {
                return u >= cumsum_last && u < cumsum;
            };

            std::size_t sample_index = 0;
            std::size_t selection_index = 0;
            for(auto u : U) {
                const int drawn = indices->at(sample_index);
                while(!in_range(u) || last_drawn == drawn) {
                    ++sample_index;
                    cumsum_last = cumsum;
                    cumsum += tau[sample_index];
                }
                selection[selection_index] = drawn;
                if(!ant_model.isSampleGood(model_, selection)){
                    continue;
                }

                ++selection_index;
                if(selection_index == model_size)
                    return;

                last_drawn = drawn;
            }
            return ant_model.isSampleGood(model_, selection);
        };

        std::size_t       skipped = 0;
        const std::size_t max_skip = max_iterations_ * 10;
        for(std::size_t iteration = 0 ; iteration < k ; ) {
            if(skipped >= max_skip)
                break;

            /// 1. update the cumulative sum vector for randomized drawing
            updateU();

            /// 2. draw with respect to the pheromone levels
            if(!draw()) {
                PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No samples could be selected!\n");
                break;
            }

            if(!sac_model_->computeModelCoefficients(selection, model_coefficients)) {
                ++skipped;
                continue;
            }

            n_inliers_count = sac_model_->countWithinDistance (model_coefficients, threshold_);
            n_inliers_mean = iteration * n_inliers_mean + n_inliers_count;
            n_inliers_mean /= static_cast<double>(iteration + 1);

            // Better match ?
            if (n_inliers_count > n_best_inliers_count)
            {
                n_best_inliers_count = n_inliers_count;

                // Save the current model/inlier/coefficients selection as being the best so far
                model_              = selection;
                model_coefficients_ = model_coefficients;

                // Compute the k parameter (k=log(z)/log(1-w^n))
                double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
                double p_no_outliers = 1.0 - pow (w, static_cast<double> (selection.size ()));
                p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
                p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
                k = log_probability / log (p_no_outliers);
            }

            updateTau(n_inliers_count);

            ++iterations_;
            PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).\n", iterations_, k, n_inliers_count, n_best_inliers_count);
            if (iterations_ > max_iterations_)
            {
                PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n");
                break;
            }
        }

        if (model_.empty ())
        {
            inliers_.clear ();
            return (false);
        }

        // Get the set of inliers that correspond to the best model found so far
        sac_model_->selectWithinDistance (model_coefficients_, threshold_, inliers_);
        return (true);
    }

protected:
    double rho_;
    double alpha_;
    double theta_;
    std::default_random_engine             rne_;
    std::uniform_real_distribution<double> distribution_;
};

}

#endif // ANTSAC_HPP
