#ifndef SAC_HPP
#define SAC_HPP

#include <memory>
#include <csapex_sample_consensus/models/sac_model.hpp>

namespace csapex_sample_consensus {
struct Parameters {
    double           model_search_distance        = 0.1;    /// samples in this distance are considered for fitting
    double           maximum_mean_model_distance  = 0.05;   /// the maximum allowed mean distance
    std::size_t      maximum_iterations           = 5000;   /// mean distance to the model
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
    std::vector<int> indices_shuffled_;

};
}

#endif // SAC_HPP
