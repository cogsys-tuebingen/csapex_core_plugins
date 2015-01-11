#ifndef SET_TIMESTAMP_H_
#define SET_TIMESTAMP_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class SetTimeStamp : public Node
{
public:
    SetTimeStamp();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_;
    Input* input_time_;
    Input* input_frame_;
    Output* output_;
};

}

#endif // SET_TIMESTAMP_H_
