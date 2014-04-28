#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ThresholdOutlierRemoval : public Node
{
public:
    ThresholdOutlierRemoval();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn*  input_;
    ConnectorIn*  thresholds_;
    ConnectorOut* output_;


};
}
#endif // STATISTICAL_OUTLIER_REMOVAL_H
