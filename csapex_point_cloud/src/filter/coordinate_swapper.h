#ifndef COORDINATE_SWAPPER_H
#define COORDINATE_SWAPPER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class CoordinateSwapper : public Node
{
public:
    CoordinateSwapper();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn*  input_;
    ConnectorOut* output_;


};
}
#endif // COORDINATE_SWAPPER_H
