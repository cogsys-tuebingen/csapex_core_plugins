#ifndef SAC_HPP
#define SAC_HPP

#include <csapex_sample_consensus/models/sac_model.hpp>
#include <memory>

namespace csapex_sample_consensus
{
struct Parameters
{
    double model_search_distance = 0.1;  /// samples in this distance are considered for fitting
    double mean_model_distance = 0.05;   /// the maximum allowed mean distance
    bool use_mean_model_distance = false;
    int maximum_iterations = 5000;  /// mean distance to the model
    bool optimize_model_coefficients = false;
    double model_validation_ratio = 0.0;  /// [0.0, 1.0], where 0.0 means the same as 1.0

    void assign(const Parameters& params)
    {
        model_search_distance = params.model_search_distance;
        model_validation_ratio = params.model_validation_ratio;
        mean_model_distance = params.mean_model_distance;
        maximum_iterations = params.maximum_iterations;
        use_mean_model_distance = params.use_mean_model_distance;
        optimize_model_coefficients = params.optimize_model_coefficients;
    }
};

template <typename PointT>
class SampleConsensus
{
public:
    using Ptr = std::shared_ptr<SampleConsensus>;
    using Model = models::Model<PointT>;

    SampleConsensus(const std::vector<int>& indices) : indices_(indices)
    {
    }

    virtual bool computeModel(typename Model::Ptr& model) = 0;

    virtual void setIndices(const std::vector<int>& indices)
    {
        indices_ = indices;
    }

    inline const std::vector<int>& getIndices() const
    {
        return indices_;
    }

protected:
    std::vector<int> indices_;
};
}  // namespace csapex_sample_consensus

#endif  // SAC_HPP
