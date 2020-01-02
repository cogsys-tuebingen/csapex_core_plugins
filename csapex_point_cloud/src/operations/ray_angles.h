#ifndef RAYANGLES_H
#define RAYANGLES_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <opencv2/core/core.hpp>

namespace csapex
{
class RayAngles : public Node
{
public:
    RayAngles();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    template <class PointT>
    void processCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    template <class PointT>
    void doProcess3D(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    void doProcess2D(pcl::PointCloud<pcl::PointXY>::ConstPtr cloud);

    enum RefAxis
    {
        X,
        Y,
        Z
    };

private:
    Input* input_;
    Output* output_general_;
    Output* output_component_a_;
    Output* output_component_b_;

    cv::Mat angles_general_;
    cv::Mat angles_component_a_;
    cv::Mat angles_component_b_;
};

}  // namespace csapex

#endif  // RAYANGLES_H
