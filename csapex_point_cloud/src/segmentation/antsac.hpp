#ifndef ANTSAC_HPP
#define ANTSAC_HPP

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model.h>

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
  typedef typename SampleConsensusModel<PointT>::Ptr SampleConsensusModelPtr;

  public:
    typedef boost::shared_ptr<AntSampleConsensus> Ptr;
    typedef boost::shared_ptr<const AntSampleConsensus> ConstPtr;

    using SampleConsensus<PointT>::max_iterations_;
    using SampleConsensus<PointT>::threshold_;
    using SampleConsensus<PointT>::iterations_;
    using SampleConsensus<PointT>::sac_model_;
    using SampleConsensus<PointT>::model_;
    using SampleConsensus<PointT>::model_coefficients_;
    using SampleConsensus<PointT>::inliers_;
    using SampleConsensus<PointT>::probability_;

    /** \brief RANSAC (RAndom SAmple Consensus) main constructor
      * \param[in] model a Sample Consensus model
      */
    AntSampleConsensus (const SampleConsensusModelPtr &model)
      : SampleConsensus<PointT> (model)
    {
      // Maximum number of trials before we give up.
      max_iterations_ = 10000;
    }

    /** \brief RANSAC (RAndom SAmple Consensus) main constructor
      * \param[in] model a Sample Consensus model
      * \param[in] threshold distance to model threshold
      */
    AntSampleConsensus (const SampleConsensusModelPtr &model, double threshold)
      : SampleConsensus<PointT> (model, threshold)
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
          PCL_ERROR ("[pcl::RandomSampleConsensus::computeModel] No threshold set!\n");
          return (false);
        }

        iterations_ = 0;
        int n_best_inliers_count = -INT_MAX;
        double k = 1.0;

        std::vector<int> selection;
        Eigen::VectorXf model_coefficients;

        double log_probability  = log (1.0 - probability_);
        double one_over_indices = 1.0 / static_cast<double> (sac_model_->getIndices ()->size ());

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
};

}

#endif // ANTSAC_HPP
