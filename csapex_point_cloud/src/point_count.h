#ifndef POINT_COUNT_H
#define POINT_COUNT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class PointCount : public Node
{
public:
    PointCount();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    ConnectorIn* input_;

public:
    boost::signals2::signal<void(int)> display_request;
};

}

#endif // POINT_COUNT_H
