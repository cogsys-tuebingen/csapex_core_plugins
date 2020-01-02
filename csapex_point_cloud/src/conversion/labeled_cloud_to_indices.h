#ifndef LABELED_CLOUD_TO_INDICES_H
#define LABELED_CLOUD_TO_INDICES_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class LabeledCloudToIndices : public csapex::Node
{
public:
    LabeledCloudToIndices();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input* input_;
    Output* output_;
};
}  // namespace csapex
#endif  // LABELED_CLOUD_TO_INDICES_H
