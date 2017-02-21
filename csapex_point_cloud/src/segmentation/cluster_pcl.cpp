#include "cluster_pcl.h"

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
        {"PCL_REGION_GROWING", (int) Method::PCL_REGION_GROWING},
        {"PCL_NORMAL", (int) Method::PCL_NORMAL},
        {"PCL_POLAR", (int) Method::PCL_POLAR},
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("method", methods, (int) Method::PCL_EUCLIDEAN),
                            [this](param::Parameter* p) {method_ = static_cast<Method>(p->as<int>());});

    parameters.addParameter(param::ParameterFactory::declareRange("cluster tolerance", 0.001, 2.0, 0.02, 0.001), cluster_tolerance_);
    parameters.addParameter(param::ParameterFactory::declareRange("minimum cluster size", 0, 20000, 100, 1),          cluster_min_size_);
    parameters.addParameter(param::ParameterFactory::declareRange("maximum cluster size", 0, 100000, 25000, 1),       cluster_max_size_);

    parameters.addConditionalParameter(param::ParameterFactory::declareRange("normal est. k nearest", 1, 500, 50, 1),
                                       [this]{return method_ == Method::PCL_NORMAL || method_ == Method::PCL_REGION_GROWING;},
                                       normal_estimator_k_nearest_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("normal est. radius", 0.001, 2.0, 0.02, 0.001),
                                       [this]{return method_ == Method::PCL_REGION_GROWING;},
                                       normal_estimator_radius_);

    parameters.addConditionalParameter(param::ParameterFactory::declareRange("region growing neighbours", 1, 500, 50, 1),
                                       [this]{return method_ == Method::PCL_REGION_GROWING;},
                                       region_growing_neighbours_);

    parameters.addConditionalParameter(param::ParameterFactory::declareAngle("region growing smoothness", 3.0 / 180.0 * M_PI),
                                       [this]{return method_ == Method::PCL_REGION_GROWING;},
                                       region_growing_smoothness_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("region growing threshold", 0.0, 5.0, 1.0, 0.001),
                                       [this]{return method_ == Method::PCL_REGION_GROWING;},
                                       region_growing_curvature_threshold_);

    parameters.addConditionalParameter(param::ParameterFactory::declareRange("normal small radius", 0.001, 2.0, 0.02, 0.001),
                                       [this]{return method_ == Method::PCL_NORMAL;},
                                       normal_small_radius_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("normal large radius", 0.001, 2.0, 0.02, 0.001),
                                       [this]{return method_ == Method::PCL_NORMAL;},
                                       normal_large_radius_);

    parameters.addConditionalParameter(param::ParameterFactory::declareAngle("opening_angle", 0.001),
                                       [this](){return method_ == Method::PCL_POLAR;} ,
                                        polar_opening_angle_);
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
    case Method::PCL_POLAR:
        cluster_indices = pclPolar<PointT>(cloud);
        break;

    default:
        throw std::runtime_error("unknown method");
    }

    msg::publish<GenericVectorMessage, pcl::PointIndices >(out_, cluster_indices);

    std::string text_msg("Found clusters: ");
    text_msg += std::to_string(cluster_indices->size());
    msg::publish(out_debug_, text_msg);
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
    ec.setClusterTolerance (cluster_tolerance_); // 2cm

    ec.setMinClusterSize (cluster_min_size_);
    ec.setMaxClusterSize (cluster_max_size_);
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

template <class PointT>
std::shared_ptr<std::vector<pcl::PointIndices> >
ClusterPointcloud::pclRegionGrowing(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
}

template <class PointT>
std::shared_ptr<std::vector<pcl::PointIndices> >
ClusterPointcloud::pclNormal(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
}

template <class PointT>
std::shared_ptr<std::vector<pcl::PointIndices> >
ClusterPointcloud::pclPolar(typename pcl::PointCloud<PointT>::ConstPtr cloud)
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
        ec.setClusterTolerance (cluster_tolerance_);
        ec.setOpeningAngle(polar_opening_angle_);

        ec.setMinClusterSize (cluster_min_size_);
        ec.setMaxClusterSize (cluster_max_size_);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (*cluster_indices);
    }

    return cluster_indices;
}


