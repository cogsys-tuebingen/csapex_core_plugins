#ifndef LABEL_POINTCLOUD_H
#define LABEL_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ColorLabeledPointCloud : public csapex::Node
{
public:
    ColorLabeledPointCloud();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input*  input_;
    Output* output_;
};
}
#endif // LABEL_POINTCLOUD_H
