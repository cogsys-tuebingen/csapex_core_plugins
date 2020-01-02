#ifndef POINTCLOUD_TO_INTENSITYIMAGE_H
#define POINTCLOUD_TO_INTENSITYIMAGE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class PointCloudToIntensityImage : public Node
{
public:
    PointCloudToIntensityImage();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    void inputCloudImpl(typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);

private:
    Input* input_;
    Output* output_;
    bool skip_invalid_;
};

}  // namespace csapex

#endif  // POINTCLOUD_TO_INTENSITYIMAGE_H
