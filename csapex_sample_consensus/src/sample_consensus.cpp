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

    out_models_  = node_modifier.addOutput<GenericVectorMessage, ModelMessage >("Models");
    out_indices_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Model Points");
}

template <class PointT>
void SampleConsensus::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::shared_ptr<std::vector<pcl::PointIndices> > out_indices(new std::vector<pcl::PointIndices>);
    std::shared_ptr<std::vector<ModelMessage> >      out_models(new std::vector<ModelMessage>);


    typename sample_consensus::SampleConsensusModel<PointT>::Ptr model(new sample_consensus::ModelPlane<PointT>(cloud));
    typename sample_consensus::Ransac<PointT>::Ptr sac(new sample_consensus::Ransac<PointT>(cloud->size(), typename sample_consensus::Ransac<PointT>::Parameters()));

    sac->computeModel(model);

    if(model) {
        pcl::PointIndices indices;
        indices.header = cloud->header;
        model->getInliers(0.1f, indices.indices);
        out_indices->emplace_back(indices);
    }

    msg::publish<GenericVectorMessage, pcl::PointIndices>(out_indices_, out_indices);
    msg::publish<GenericVectorMessage, ModelMessage>(out_models_, out_models);
}

//template <class PointT>
//void SacFit2::estimateNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud,
//                             pcl::PointCloud<pcl::Normal>::Ptr normals)
//{
//    typename pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
//    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

//    normal_estimation.setSearchMethod (tree);
//    normal_estimation.setInputCloud (cloud);
//    normal_estimation.setKSearch (50);
//    normal_estimation.compute (*normals);
//}

//bool SacFit2::need_normals()
//{
//    switch(model_type_){
//    case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
//        return true;
//    case pcl::SACMODEL_NORMAL_PLANE:
//        return true;
//    case pcl::SACMODEL_CONE:
//        return true;
//    default:
//        return false;
//    }
//}
