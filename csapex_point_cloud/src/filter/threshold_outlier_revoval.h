#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class ThresholdOutlierRemoval : public Node
{
public:
    ThresholdOutlierRemoval();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_;
    Input* thresholds_;
    Output* output_;
};
}  // namespace csapex
#endif  // STATISTICAL_OUTLIER_REMOVAL_H
