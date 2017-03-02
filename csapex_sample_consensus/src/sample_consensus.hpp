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
        std::map<std::string, std::pair<int, bool>> termination_criteria =
        {
            {"MAX_ITERATION", {csapex_sample_consensus::Parameters::MAX_ITERATION, true}},
            {"MIN_DISTANCE", {csapex_sample_consensus::Parameters::MIN_DISTANCE, true}},
            {"MAX_RETRY", {csapex_sample_consensus::Parameters::MAX_RETRY, true}}
        };

        parameters.addParameter(param::ParameterFactory::declareParameterBitSet("termination criteria",termination_criteria),
                                termination_criteria_);
        parameters.addParameter(param::ParameterFactory::declareRange("model search distance", 0.0, 10.0, 0.1, 0.01),
                                model_search_distance_);
        parameters.addParameter(param::ParameterFactory::declareRange("maximum mean model distance", 0.0, 10.0, 0.05, 0.01),
                                maximum_mean_model_distance_);
        parameters.addParameter(param::ParameterFactory::declareRange("maximum iterations", 1, 100000, 100, 1),
                                maximum_iterations_);
        parameters.addParameter(param::ParameterFactory::declareRange("maximum retries", 1, 100000, 100, 1),
                                maximum_retries_);
        parameters.addParameter(param::ParameterFactory::declareRange("minimum inlier percentage", 0.0, 100.0, 50.0, 0.1),
                                minimum_inlier_percentage_);

    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_cloud_           = node_modifier.addInput<PointCloudMessage>("PointCloud");
        in_indices_         = node_modifier.addOptionalInput<GenericVectorMessage, pcl::PointIndices>("Indices"); // optional input

        out_models_         = node_modifier.addOutput<GenericVectorMessage, ModelMessage >("Models");
        out_inlier_indices_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Model Points");
        out_outlier_indices_= node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Rejected Points");
    }

protected:
    int         termination_criteria_;
    double      model_search_distance_;
    double      maximum_mean_model_distance_;
    int         maximum_iterations_;
    int         maximum_retries_;
    double      minimum_inlier_percentage_;

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


    void fillParamterObject(csapex_sample_consensus::Parameters &params)
    {
        params.termination_criteria        = termination_criteria_;
        params.model_search_distance       = model_search_distance_;
        params.maximum_mean_model_distance = maximum_mean_model_distance_;
        params.maximum_iterations          = maximum_iterations_;
        params.maximum_retries             = maximum_retries_;
        params.minimum_inlier_percentage   = minimum_inlier_percentage_ * 0.01; /// shift the representation to [0.0, 1.0]
    }

};
}

#endif // SAMPLE_CONSENSUS_HPP
