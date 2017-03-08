#ifndef SAMPLE_CONSENSUS_HPP
#define SAMPLE_CONSENSUS_HPP

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>

#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_value_message.hpp>

#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/model_message.h>
#include <csapex_point_cloud/indeces_message.h>

#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#include <pcl/point_representation.h>

#include <boost/mpl/for_each.hpp>

#include <csapex_sample_consensus/csapex_sample_consensus.hpp>

namespace csapex {
using namespace connection_types;

class SampleConsensus : public csapex::Node
{
public:
    SampleConsensus() = default;

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("model search distance", 0.0, 10.0, 0.1, 0.001),
                                sac_parameters_.model_search_distance);
        parameters.addParameter(param::ParameterFactory::declareBool("terminate one mean model distance", false),
                                sac_parameters_.use_mean_model_distance);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("maximum mean model distance", 0.0, 10.0, 0.05, 0.01),
                                           [this](){return sac_parameters_.use_mean_model_distance;},
                                           sac_parameters_.mean_model_distance);
        parameters.addParameter(param::ParameterFactory::declareRange("maximum iterations", 1, 100000, 100, 1),
                                sac_parameters_.maximum_iterations);

        parameters.addParameter(param::ParameterFactory::declareRange("minimum model cloud size", 1, 100000, 1000, 1),
                                minimum_model_cloud_size_);

        parameters.addParameter(param::ParameterFactory::declareBool("find multiple models", false),
                                fit_multiple_models_);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("minimum residual cloud size", 0, 1000, 100, 1),
                                           [this](){return fit_multiple_models_;},
                                minimum_residual_cloud_size_);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("maximum model count", -1, 100, 5, 1),
                                           [this](){return fit_multiple_models_;},
                                maximum_model_count_);

    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_cloud_           = node_modifier.addInput<PointCloudMessage>("PointCloud");
        in_indices_         = node_modifier.addOptionalInput<PointIndecesMessage>("Indices"); // optional input

        out_models_         = node_modifier.addOutput<GenericVectorMessage, ModelMessage >("Models");
        out_inlier_indices_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Model Points");
        out_outlier_indices_= node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Rejected Points");
    }

protected:
    csapex_sample_consensus::Parameters sac_parameters_;

    int         termination_criteria_;
    int         minimum_fit_size_;
    int         minimum_model_cloud_size_;

    bool        fit_multiple_models_;
    int         minimum_residual_cloud_size_;
    int         maximum_model_count_;

    Input*  in_cloud_;
    Input*  in_indices_;
    Output* out_models_;
    Output* out_inlier_indices_;
    Output* out_outlier_indices_;

    template<typename PointT>
    typename csapex_sample_consensus::SampleConsensusModel<PointT>::Ptr getModel(typename pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
        return typename csapex_sample_consensus::ModelPlane<PointT>::Ptr(new csapex_sample_consensus::ModelPlane<PointT>(cloud));
    }

    template<typename PointT>
    void getIndices(typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                    std::vector<int> &indices)
    {
        const static pcl::DefaultPointRepresentation<PointT> pr;
        const std::size_t size = cloud->size();
        indices.reserve(size);
        for(std::size_t i = 0 ; i < size; ++i) {
            if(pr.isValid(cloud->at(i))) {
                indices.emplace_back(i);
            }
        }

    }
};
}

#endif // SAMPLE_CONSENSUS_HPP
