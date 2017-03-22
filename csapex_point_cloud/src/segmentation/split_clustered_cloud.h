#ifndef SPLIT_CLUSTERED_CLOUD_H
#define SPLIT_CLUSTERED_CLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex {
class SplitClusteredCloud : public csapex::Node
{
public:
    SplitClusteredCloud();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input*  input_;
    Input* in_indices_;
    Output* output1_;
    Output* output2_;
    Output* output3_;
    Output* output4_;
};
}
#endif // SPLIT_CLUSTERED_CLOUD_H
