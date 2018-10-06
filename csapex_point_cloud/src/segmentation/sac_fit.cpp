#include "sac_fit.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/msg/indices_message.h>

/// #include "sac_segmentation.hpp"

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <boost/mpl/for_each.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf/tf.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

CSAPEX_REGISTER_CLASS(csapex::SacFit, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace std;

SacFit::SacFit()
{
}

void SacFit::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareRange("iterations", 1, 20000, 5000, 200), max_iterations_);
    parameters.addParameter(param::factory::declareRange("min inliers", 5, 20000, 100, 100), min_inliers_);
    parameters.addParameter(param::factory::declareRange("normal distance weight", 0.0, 2.0, 0.085, 0.001), normal_distance_weight_);
    parameters.addParameter(param::factory::declareRange("distance threshold", 0.0, 2.0, 0.009, 0.001), distance_threshold_);
    parameters.addParameter(param::factory::declareValue<double>("model_main_axis_x", 0), model_main_axis_x_);
    parameters.addParameter(param::factory::declareValue<double>("model_main_axis_y", 0), model_main_axis_y_);
    parameters.addParameter(param::factory::declareValue<double>("model_main_axis_z", 0), model_main_axis_z_);
    parameters.addParameter(param::factory::declareValue<double>("model_angle_offset", 0), model_angle_offset_);
    parameters.addParameter(param::factory::declareRange("sphere min radius", 0.0, 2.0, 0.02, 0.005), sphere_r_max_);
    parameters.addParameter(param::factory::declareRange("sphere max radius", 0.0, 2.0, 0.8, 0.005), sphere_r_min_);

    parameters.addParameter(param::factory::declareBool("from normals", false), from_normals_);

    parameters.addParameter(param::factory::declareBool("optimize coefficients", true), optimize_coefficients_);

    std::map<std::string, int> model_types = { { "CIRCLE2D", pcl::SACMODEL_CIRCLE2D },
                                               { "CIRCLE3D", pcl::SACMODEL_CIRCLE3D },
                                               { "CONE", (int)pcl::SACMODEL_CONE },
                                               { "CYLINDER", (int)pcl::SACMODEL_CYLINDER },
                                               { "LINE", pcl::SACMODEL_LINE },
                                               { "NORMAL_PARALLEL_PLANE", (int)pcl::SACMODEL_NORMAL_PARALLEL_PLANE },
                                               { "NORMAL_PLANE", pcl::SACMODEL_NORMAL_PLANE },
                                               { "PARALLEL_LINE", pcl::SACMODEL_PARALLEL_LINE },
                                               { "PARALLEL_LINES", pcl::SACMODEL_PARALLEL_LINES },
                                               { "PARALLEL_PLANE", pcl::SACMODEL_PARALLEL_PLANE },
                                               { "PLANE", (int)pcl::SACMODEL_PLANE },
                                               { "PERPENDICULAR_PLANE", pcl::SACMODEL_PERPENDICULAR_PLANE },
                                               { "SPHERE", (int)pcl::SACMODEL_SPHERE },
                                               { "STICK", pcl::SACMODEL_STICK },
                                               { "TORUS", pcl::SACMODEL_TORUS } };

    parameters.addParameter(param::factory::declareParameterSet<int>("models", model_types, (int)pcl::SACMODEL_PLANE), model_type_);

    std::map<std::string, int> ransac_types = {
        { "LMEDS", pcl::SAC_LMEDS },   { "MLESAC", pcl::SAC_MLESAC }, { "MSAC", pcl::SAC_MSAC },       { "PROSAC", pcl::SAC_PROSAC },
        { "RANSAC", pcl::SAC_RANSAC }, { "RMSAC", pcl::SAC_RMSAC },   { "RRANSAC", pcl::SAC_RRANSAC },
    };

    parameters.addParameter(param::factory::declareParameterSet<int>("ransac type", ransac_types, (int)pcl::SAC_RANSAC), ransac_type_);
}

void SacFit::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor(PointCloudMessage::Dispatch<SacFit>(this, msg), msg->value);
}

void SacFit::setup(NodeModifier& node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    in_indices_ = node_modifier.addOptionalInput<GenericVectorMessage, pcl::PointIndices>("Indices");  // optional input

    out_models_ = node_modifier.addOutput<GenericVectorMessage, ModelMessage>("Models");
    out_indices_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Model Points");
}

template <class PointT>
void SacFit::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::shared_ptr<std::vector<pcl::PointIndices>> out_indices(new std::vector<pcl::PointIndices>);
    std::shared_ptr<std::vector<ModelMessage>> out_models(new std::vector<ModelMessage>);

    /// configure the segmentation
    pcl::SACSegmentation<PointT>* segmenter;
    bool normals_needed = need_normals();  // some SAC models need normals / access conditional
    if (from_normals_ || normals_needed) {
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal>* normal_segmenter;
        normal_segmenter = new pcl::SACSegmentationFromNormals<PointT, pcl::Normal>;
        normal_segmenter->setNormalDistanceWeight(normal_distance_weight_);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        estimateNormals<PointT>(cloud, normals);  /// input indices ??
        normal_segmenter->setInputNormals(normals);

        segmenter = normal_segmenter;
    } else {
        segmenter = new pcl::SACSegmentation<PointT>;
    }
    segmenter->setOptimizeCoefficients(optimize_coefficients_);
    segmenter->setModelType(model_type_);
    segmenter->setMethodType(ransac_type_);
    segmenter->setMaxIterations(max_iterations_);
    segmenter->setDistanceThreshold(distance_threshold_);

    //    segmenter->setRadiusLimits (sphere_r_min_, sphere_r_max_);      //
    //    circular objects / access conditional
    //    segmenter->setMinMaxOpeningAngle(sphere_r_min_, sphere_r_max_); // cones
    //    / access conditional

    if (model_main_axis_x_ != 0 || model_main_axis_y_ != 0 || model_main_axis_z_ != 0) {
        Eigen::Vector3f axis(model_main_axis_x_, model_main_axis_y_, model_main_axis_z_);
        segmenter->setAxis(axis);
        segmenter->setEpsAngle(model_angle_offset_);  /// angle to access conditional
    }

    /// let's fit
    segmenter->setInputCloud(cloud);
    if (msg::hasMessage(in_indices_)) {
        std::shared_ptr<std::vector<pcl::PointIndices> const> in_indices = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_indices_);
        for (const pcl::PointIndices& indices : *in_indices) {
            pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));
            segmenter->setIndices(indices_ptr);

            pcl::PointIndices inliers;
            segmenter->segment(inliers, *model_coefficients);
            if ((int)inliers.indices.size() > min_inliers_) {
                out_indices->emplace_back(inliers);

                ModelMessage model;
                model.coefficients = model_coefficients;
                model.probability = segmenter->getProbability();
                model.frame_id = cloud->header.frame_id;
                model.model_type = (pcl::SacModel)model_type_;
                out_models->emplace_back(model);
            }
        }
    } else {
        /// modify for more than one model
        pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices inliers;
        segmenter->segment(inliers, *model_coefficients);
        if ((int)inliers.indices.size() > min_inliers_) {
            out_indices->emplace_back(inliers);

            ModelMessage model;
            model.coefficients = model_coefficients;
            model.probability = segmenter->getProbability();
            model.frame_id = cloud->header.frame_id;
            model.model_type = (pcl::SacModel)model_type_;
            out_models->emplace_back(model);
        }
    }

    msg::publish<GenericVectorMessage, pcl::PointIndices>(out_indices_, out_indices);
    msg::publish<GenericVectorMessage, ModelMessage>(out_models_, out_models);

    delete segmenter;
}

template <class PointT>
void SacFit::estimateNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    typename pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    normal_estimation.setSearchMethod(tree);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setKSearch(50);
    normal_estimation.compute(*normals);
}

bool SacFit::need_normals()
{
    switch (model_type_) {
        case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
            return true;
        case pcl::SACMODEL_NORMAL_PLANE:
            return true;
        case pcl::SACMODEL_CONE:
            return true;
        default:
            return false;
    }
}
