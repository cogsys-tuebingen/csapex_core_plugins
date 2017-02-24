#ifndef ANTSAC_HPP
#define ANTSAC_HPP

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model.h>

#include <random>

namespace csapex {
/** \brief @b RandomSampleConsensus represents an implementation of the RANSAC (RAndom SAmple Consensus) algorithm, as
  * described in: "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and
  * Automated Cartography", Martin A. Fischler and Robert C. Bolles, Comm. Of the ACM 24: 381–395, June 1981.
  * \author Radu B. Rusu
  * \ingroup sample_consensus
  */
template <typename PointT>
class AntSampleConsensus : public pcl::SampleConsensus<PointT>
{
  typedef typename AntSampleConsensus<PointT>::Ptr SampleConsensusModelPtr;

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
    AntSampleConsensus (const SampleConsensusModelPtr &model,
                        double rho = 0.9,
                        double alpha = 0.1)
      : pcl::SampleConsensus<PointT> (model),
        rho_(rho),
        alpha_(alpha),
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
                        double alpha = 0.1)
      : pcl::SampleConsensus<PointT> (model, threshold),
        rho_(rho),
        alpha_(alpha),
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

        std::vector<int> selection;
        Eigen::VectorXf model_coefficients;



        const std::size_t size = sac_model_->getIndices()->size();
        const double log_probability  = log (1.0 - probability_);
        const double one_over_indices = 1.0 / static_cast<double> (size);

        std::vector<double> tau(size, one_over_indices);
        std::vector<double> D;

        auto update_D = [&D, &tau, this]() {

        };

        const std::size_t model_samples = sac_model_->getSampleSize();
        const auto        indices = sac_model_->getIndices();

        /// implement drawing
        /// implement update of tau
        /// the rest should be as ransac


//        std::size_t k = size;
//        math::random::Uniform<1> rng(0.0, 1.0);
//        std::vector<double> u(size, std::pow(rng.get(), 1.0 / k));
//        {
//            for(int k = u.size() - 2; k >= 0 ; --k) {
//                double u_ = std::pow(rng.get(), 1.0 / k);
//                u[k] = u[k+1] * u_;
//            }
//        }




//        virtual void
//        getSamples (int &iterations, std::vector<int> &samples)
//        {
//          // We're assuming that indices_ have already been set in the constructor
//          if (indices_->size () < getSampleSize ())
//          {
//            PCL_ERROR ("[pcl::SampleConsensusModel::getSamples] Can not select %zu unique points out of %zu!\n",
//                       samples.size (), indices_->size ());
//            // one of these will make it stop :)
//            samples.clear ();
//            iterations = INT_MAX - 1;
//            return;
//          }

//          // Get a second point which is different than the first
//          samples.resize (getSampleSize ());
//          for (unsigned int iter = 0; iter < max_sample_checks_; ++iter)
//          {
//            // Choose the random indices
//            if (samples_radius_ < std::numeric_limits<double>::epsilon ())
//                SampleConsensusModel<PointT>::drawIndexSample (samples);
//            else
//                SampleConsensusModel<PointT>::drawIndexSampleRadius (samples);

//            // If it's a good sample, stop here
//            if (isSampleGood (samples))
//            {
//              PCL_DEBUG ("[pcl::SampleConsensusModel::getSamples] Selected %zu samples.\n", samples.size ());
//              return;
//            }
//          }
//          PCL_DEBUG ("[pcl::SampleConsensusModel::getSamples] WARNING: Could not select %d sample points in %d iterations!\n", getSampleSize (), max_sample_checks_);
//          samples.clear ();
//        }



        int n_inliers_count = 0;
        unsigned skipped_count = 0;
        // supress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
        const unsigned max_skip = max_iterations_ * 10;

        // Iterate
        while (iterations_ < k && skipped_count < max_skip)
        {
          // Get X samples which satisfy the model criteria
          sac_model_->getSamples (iterations_, selection);

          if (selection.empty ())
          {
            PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No samples could be selected!\n");
            break;
          }

          // Search for inliers in the point cloud for the current plane model M
          if (!sac_model_->computeModelCoefficients (selection, model_coefficients))
          {
            //++iterations_;
            ++skipped_count;
            continue;
          }

          // Select the inliers that are within threshold_ from the model
          //sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
          //if (inliers.empty () && k > 1.0)
          //  continue;

          n_inliers_count = sac_model_->countWithinDistance (model_coefficients, threshold_);

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

          ++iterations_;
          PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).\n", iterations_, k, n_inliers_count, n_best_inliers_count);
          if (iterations_ > max_iterations_)
          {
            PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n");
            break;
          }
        }

        PCL_DEBUG ("[pcl::RandomSampleConsensus::computeModel] Model: %zu size, %d inliers.\n", model_.size (), n_best_inliers_count);

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
    std::default_random_engine             rne_;
    std::uniform_real_distribution<double> distribution_;
};

}

#endif // ANTSAC_HPP
