#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class RadiusOutlierRemoval : public Node
{
public:
    RadiusOutlierRemoval();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*  input_cloud_;
    ConnectorIn*  indeces_input_;
    ConnectorOut* output_cloud_;
    ConnectorOut* output_indeces_;

};
}
#endif // STATISTICAL_OUTLIER_REMOVAL_H
