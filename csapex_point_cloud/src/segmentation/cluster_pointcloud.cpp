#include "cluster_pointcloud.h"

#include "polar_clustering.hpp"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/profiling/interlude.hpp>
#include <csapex/profiling/timer.h>

/// SYSTEM
#include <boost/mpl/for_each.hpp>
#include <tf/tf.h>

/// PCL
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#pragma clang diagnostic ignored "-Wsign-compare"
#endif //__clang__
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#if __clang__
#pragma clang diagnostic pop
#endif //__clang__


CSAPEX_REGISTER_CLASS(csapex::ClusterPointcloud, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace std;


ClusterPointcloud::ClusterPointcloud()
{
}

void ClusterPointcloud::setupParameters(Parameterizable &parameters)
{
    std::map<std::string, int> methods = {
        {"PCL_EUCLIDEAN", (int) Method::PCL_EUCLIDEAN},
        {"POLAR", (int) Method::POLAR},
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("method", methods, (int) Method::PCL_EUCLIDEAN),
                            [this](param::Parameter* p) {method_ = static_cast<Method>(p->as<int>());});

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("ClusterTolerance", 0.001, 2.0, 0.02, 0.001), param_clusterTolerance_);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("MinClusterSize", 0, 20000, 100, 1), param_clusterMinSize_);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("MaxClusterSize", 0, 100000, 25000, 1), param_clusterMaxSize_);

    parameters.addParameter(csapex::param::ParameterFactory::declareAngle("opening_angle", 0.001), opening_angle_);
}

void ClusterPointcloud::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));

    boost::apply_visitor (PointCloudMessage::Dispatch<ClusterPointcloud>(this, msg), msg->value);
}

void ClusterPointcloud::setup(NodeModifier& node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_ = node_modifier.addOptionalInput<PointIndecesMessage>("Indices");

    out_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices >("Clusters");
    out_debug_ = node_modifier.addOutput<std::string>("Debug Info");
}

template <class PointT>
void ClusterPointcloud::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{

    std::shared_ptr<std::vector<pcl::PointIndices> > cluster_indices;

    switch(method_) {
    case Method::PCL_EUCLIDEAN:
        cluster_indices = pclEuclidean<PointT>(cloud);
        break;
    case Method::POLAR:
        cluster_indices = polar<PointT>(cloud);
        break;

    default:
        throw std::runtime_error("unknown method");
    }

    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, cluster_indices);

    std::string text_msg("Found clusters: ");
    text_msg += std::to_string(cluster_indices->size());
    msg::publish(out_debug_, text_msg);
}

namespace {
template <class PointT>
bool customRegionGrowing (const PointT& point_a,
                          const PointT& point_b,
                          float squared_distance)
{
    return false;
}
template <>
bool customRegionGrowing<pcl::PointXYZI> (const pcl::PointXYZI& point_a,
                                          const pcl::PointXYZI& point_b,
                                          float squared_distance)
{
    if (squared_distance < 10000)
    {
        if (fabs (point_a.intensity - point_b.intensity) < 8.0f)
            return true;

    } else {
        if (fabs (point_a.intensity - point_b.intensity) < 3.0f)
            return true;
    }
    return false;
}
}

template <class PointT>
std::shared_ptr<std::vector<pcl::PointIndices> >
ClusterPointcloud::polar(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    pcl::PointIndicesPtr indices;
    if(msg::isConnected(in_indices_)) {
        auto indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
        indices = indices_msg->value;
    }

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::shared_ptr<std::vector<pcl::PointIndices> > cluster_indices(new std::vector<pcl::PointIndices>);

    {
        INTERLUDE("clustering");
        PolarClustering<PointT> ec;
        ec.setClusterTolerance (param_clusterTolerance_);
        ec.setOpeningAngle(opening_angle_);

        ec.setMinClusterSize (param_clusterMinSize_);
        ec.setMaxClusterSize (param_clusterMaxSize_);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (*cluster_indices);
    }

    return cluster_indices;
}


template <class PointT>
std::shared_ptr<std::vector<pcl::PointIndices> >
ClusterPointcloud::pclEuclidean(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    typename pcl::PointCloud<PointT>::ConstPtr cloud_clean;

    pcl::PointIndicesPtr indices;
    if(msg::isConnected(in_indices_)) {
        // NOTE: not using indices here directly, because that is way slower!
        typename pcl::PointCloud<PointT>::Ptr cloud_clean_prime(new pcl::PointCloud<PointT>);
        auto indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
        indices = indices_msg->value;

        for(auto it = indices->indices.begin(); it != indices->indices.end();) {
            auto& pt = cloud->points[*it];
            if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                it = indices->indices.erase(it);
            } else {
                ++it;
            }
        }

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (cloud);
        extract.setNegative(false);
        extract.setIndices (indices);
        extract.filter (*cloud_clean_prime);

        cloud_clean = cloud_clean_prime;

    } else {
        // check for nans in cloud
        typename pcl::PointCloud<PointT>::Ptr cloud_clean_prime(new pcl::PointCloud<PointT>);
        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud<PointT>(*cloud, *cloud_clean_prime, nan_indices);
        cloud_clean_prime->is_dense = false;

        auto& pts = cloud_clean_prime->points;
        for(auto it = pts.begin(); it != cloud_clean_prime->end();) {
            auto& pt = *it;
            if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                it = pts.erase(it);
            } else {
                ++it;
            }
        }

        cloud_clean = cloud_clean_prime;
    }


    // from http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_clean);

    std::shared_ptr<std::vector<pcl::PointIndices> > cluster_indices(new std::vector<pcl::PointIndices>);
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (param_clusterTolerance_); // 2cm

    ec.setMinClusterSize (param_clusterMinSize_);
    ec.setMaxClusterSize (param_clusterMaxSize_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_clean);
    // NOTE: not using indices here directly, because that is way slower!
    // ec.setIndices(indices);
    ec.extract (*cluster_indices);


    if(indices) {
        const std::vector<int>& idx = indices->indices;
        for(pcl::PointIndices& in : *cluster_indices) {
            for(int& i : in.indices) {
                i = idx[i];
            }
        }
    }

    return cluster_indices;
}
