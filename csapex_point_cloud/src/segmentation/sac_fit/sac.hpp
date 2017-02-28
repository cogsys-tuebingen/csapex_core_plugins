#ifndef SAC_HPP
#define SAC_HPP

#include <memory>
#include "sac_model.hpp"

namespace sample_consensus {
template<typename PointT>
class SampleConsensus  {
public:
    using Ptr = std::shared_ptr<SampleConsensus>;
    using Model = SampleConsensusModel<PointT>;

    struct Parameters {
        float            threshold = 0.1;
        std::size_t      maximum_iterations = 5000;
        double           probability = 0.99;
    };

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

    void setIndices(const std::vector<int> &indices)
    {
        indices_ = indices;
    }

protected:
    std::vector<int> indices_;

};
}

#endif // SAC_HPP
