#ifndef ESTIMATE_NORMALS_H
#define ESTIMATE_NORMALS_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/normals_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>

/// SYSTEM
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/search/impl/kdtree.hpp>

using namespace csapex::connection_types;

namespace csapex
{

class EstimateNormals : public Node
{
public:
    EstimateNormals()
    {

    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<PointCloudMessage>("cloud");
        output_ = node_modifier.addOutput<NormalsMessage>("normals");
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(param::ParameterFactory::declareBool("use_pca_based_estimation", false),
                                use_pca_);
        parameters.addConditionalParameter(param::ParameterFactory::declareBool("use_open_mp", false),
                                           [this](){return use_pca_;},
                                           use_omp_);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("search_radius",
                                                                                 0.01, 1.0, 0.03, 0.01),
                                           [this](){return use_pca_;},
                                           search_radius_);

        parameters.addConditionalParameter(param::ParameterFactory::declareRange("max_depth_change_factor",
                                                                                  0.0, 0.5, 0.02, 0.001),
                                           [this](){return !use_pca_;},
                                           max_depth_change_factor_);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("normal_smoothing_size",
                                                                                 0.0, 50.0, 10.0, 0.1),
                                           [this](){return !use_pca_;},
                                           normal_smoothing_size_);

        typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> T;

        std::map<std::string, int> methods {
            {"COVARIANCE_MATRIX", T::COVARIANCE_MATRIX},
            {"AVERAGE_3D_GRADIENT", T::AVERAGE_3D_GRADIENT},
            {"AVERAGE_DEPTH_CHANGE", T::AVERAGE_DEPTH_CHANGE},
            {"SIMPLE_3D_GRADIENT", T::SIMPLE_3D_GRADIENT}
        };
        parameters.addConditionalParameter(param::ParameterFactory::declareParameterSet("method", methods,
                                                                                        (int) T::AVERAGE_3D_GRADIENT),
                                           [this](){return !use_pca_;},
                                           method_);
    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

        boost::apply_visitor (PointCloudMessage::Dispatch<EstimateNormals>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        NormalsMessage::Ptr out_msg(new NormalsMessage(cloud->header.frame_id, cloud->header.stamp));
        pcl::PointCloud<pcl::Normal>::Ptr msg(new pcl::PointCloud<pcl::Normal>);
        out_msg->value = msg;

        if(use_pca_) {
            typename pcl::NormalEstimation<PointT, pcl::Normal>::Ptr ne;
            if(use_omp_) {
                ne.reset(new pcl::NormalEstimationOMP<PointT, pcl::Normal>);
            } else {
                ne.reset(new pcl::NormalEstimation<PointT, pcl::Normal>);
            }

            ne->setInputCloud (cloud);

            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
            ne->setSearchMethod (tree);

            // Use all neighbors in a sphere of radius 3cm
            ne->setRadiusSearch (search_radius_);

            // Compute the features
            ne->compute (*msg);

        }  else {
            typedef pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> N;
            N ne;
            ne.setNormalEstimationMethod (static_cast<typename N::NormalEstimationMethod>(method_));
            ne.setMaxDepthChangeFactor(max_depth_change_factor_);
            ne.setNormalSmoothingSize(normal_smoothing_size_);
            ne.setInputCloud(cloud);
            ne.compute(*msg);
        }

        msg::publish(output_, out_msg);
    }

private:
    Input*  input_;
    Output* output_;

    int    method_;
    double max_depth_change_factor_;
    double normal_smoothing_size_;
    bool   use_pca_;
    bool   use_omp_;
    double search_radius_;

};

}

CSAPEX_REGISTER_CLASS(csapex::EstimateNormals, csapex::Node)

#endif // ESTIMATE_NORMALS_H
