#include "sample_consensus.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex/msg/generic_value_message.hpp>


/// POINT CLOUD
#include "sac.hpp"
#include "ransac.hpp"
#include "antsac.hpp"

#include "sac_model.hpp"
#include "sac_model_plane.hpp"
#include "sac_model_from_normals.hpp"
#include "sac_model_normal_plane.hpp"


/// SYSTEM
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <tf/tf.h>
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::SampleConsensus, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace std;



SampleConsensus::SampleConsensus()
{
}

void SampleConsensus::setupParameters(Parameterizable &parameters)
{

}


void SampleConsensus::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<SampleConsensus>(this, msg), msg->value);
}

void SampleConsensus::setup(NodeModifier& node_modifier)
{
    in_cloud_    = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_  = node_modifier.addOptionalInput<GenericVectorMessage, pcl::PointIndices>("Indices"); // optional input

    out_models_         = node_modifier.addOutput<GenericVectorMessage, ModelMessage >("Models");
    out_inlier_indices_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Model Points");
    out_outlier_indices_= node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Rejected Points");
}

template <class PointT>
void SampleConsensus::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::shared_ptr<std::vector<pcl::PointIndices> > out_inliers(new std::vector<pcl::PointIndices>);
    std::shared_ptr<std::vector<pcl::PointIndices> > out_outliers(new std::vector<pcl::PointIndices>);
    std::shared_ptr<std::vector<ModelMessage> >      out_models(new std::vector<ModelMessage>);

    typename sample_consensus::SampleConsensusModel<PointT>::Ptr model(new sample_consensus::ModelPlane<PointT>(cloud));
    typename sample_consensus::Ransac<PointT>::Ptr sac(new sample_consensus::Ransac<PointT>(cloud->size(), typename sample_consensus::Ransac<PointT>::Parameters()));

    sac->computeModel(model);

    if(model) {
        pcl::PointIndices outliers;
        pcl::PointIndices inliers;
        inliers.header = cloud->header;
        outliers.header = cloud->header;
        model->getInliersAndOutliers(0.1f, inliers.indices, outliers.indices);
        out_inliers->emplace_back(inliers);
        out_outliers->emplace_back(outliers);
    }

    msg::publish<GenericVectorMessage, pcl::PointIndices>(out_inlier_indices_, out_inliers);
    msg::publish<GenericVectorMessage, pcl::PointIndices>(out_outlier_indices_, out_outliers);
    msg::publish<GenericVectorMessage, ModelMessage>(out_models_, out_models);
}
