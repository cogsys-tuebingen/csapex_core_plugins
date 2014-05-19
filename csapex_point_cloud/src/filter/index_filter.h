#ifndef INDEX_FILTER_H
#define INDEX_FILTER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class IndexFilter : public Node
{
public:
    IndexFilter();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

protected:
    ConnectorIn*  input_cloud_;
    ConnectorIn*  indeces_input_;
    ConnectorOut* output_cloud_;

};
}
#endif // INDEX_FILTER_H
