#ifndef POINT_COUNT_H
#define POINT_COUNT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class TransformCloud : public Node
{
public:
    TransformCloud();

    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_cloud_;
    Input* input_transform_;
    Output* output_;
};

}  // namespace csapex

#endif  // POINT_COUNT_H
