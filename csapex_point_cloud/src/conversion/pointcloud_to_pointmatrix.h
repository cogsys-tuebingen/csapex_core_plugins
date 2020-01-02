#ifndef TO_POINT_MATRIX_H
#define TO_POINT_MATRIX_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class PointCloudToPointMatrix : public csapex::Node
{
public:
    PointCloudToPointMatrix();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input* input_;
    Output* output_;
    Output* mask_;
};
}  // namespace csapex
#endif  // TO_POINT_MATRIX_H
