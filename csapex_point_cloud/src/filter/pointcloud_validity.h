#ifndef UNDEFINED_VALUE_MASK_H
#define UNDEFINED_VALUE_MASK_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class PointCloudValidity : public csapex::Node
{
public:
    PointCloudValidity();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*  input_;
    ConnectorOut* mask_;
    ConnectorOut* index_;
};
}

#endif // UNDEFINED_VALUE_MASK_H
