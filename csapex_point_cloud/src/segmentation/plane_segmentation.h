#ifndef PLANE_SEGMENTATION_H
#define PLANE_SEGMENTATION_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <pcl/point_cloud.h>

namespace csapex
{
class PlaneSegmentation : public csapex::Node
{
public:
    PlaneSegmentation() = default;

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input* input_cloud_;

    Output* output_normals_;
};
}  // namespace csapex

#endif  // PLANE_SEGMENTATION_H
