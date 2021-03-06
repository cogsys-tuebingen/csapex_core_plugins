#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class RadiusOutlierRemoval : public Node
{
public:
    RadiusOutlierRemoval();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input* input_cloud_;
    Input* indices_input_;
    Output* output_cloud_;
    Output* output_indices_;
};
}  // namespace csapex
#endif  // STATISTICAL_OUTLIER_REMOVAL_H
