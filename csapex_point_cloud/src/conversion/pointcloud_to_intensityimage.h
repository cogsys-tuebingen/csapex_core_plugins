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

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    void inputCloudImpl(typename pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);

private:
    Input* input_;
    Output* output_;
};

}

#endif // POINTCLOUD_TO_INTENSITYIMAGE_H
