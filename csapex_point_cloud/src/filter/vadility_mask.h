#ifndef UNDEFINED_VALUE_MASK_H
#define UNDEFINED_VALUE_MASK_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class VadilityMask : public csapex::Node
{
public:
    VadilityMask();

    virtual void process();
    virtual void setup();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*  input_;
    ConnectorOut* output_;
};
}

#endif // UNDEFINED_VALUE_MASK_H
