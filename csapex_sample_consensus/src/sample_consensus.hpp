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
        /// general sample consenus parameters
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
    Input*  in_cloud_;
    Input*  in_indices_;
    Output* out_models_;
    Output* out_inlier_indices_;
    Output* out_outlier_indices_;


};
}

#endif // SAMPLE_CONSENSUS_HPP
