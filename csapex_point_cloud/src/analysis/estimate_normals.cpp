#ifndef ESTIMATE_NORMALS_H
#define ESTIMATE_NORMALS_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/normals_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
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

        bool dense = cloud->width > 1 && cloud->height > 1;
        if(!dense) {
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            ne.setInputCloud (cloud);

            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
            ne.setSearchMethod (tree);

            // Use all neighbors in a sphere of radius 3cm
            ne.setRadiusSearch (0.03);

            // Compute the features
            ne.compute (*msg);

        }  else {
            pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
            ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
            ne.setMaxDepthChangeFactor(0.02f);
            ne.setNormalSmoothingSize(10.0f);
            ne.setInputCloud(cloud);
            ne.compute(*msg);
        }

        msg::publish(output_, out_msg);
    }

private:
    Input*  input_;
    Output* output_;


};

}

CSAPEX_REGISTER_CLASS(csapex::EstimateNormals, csapex::Node)

#endif // ESTIMATE_NORMALS_H
