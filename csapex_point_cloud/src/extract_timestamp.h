#ifndef EXTRACT_TIMESTAMP_CLOUD_H_
#define EXTRACT_TIMESTAMP_CLOUD_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class ExtractTimeStampCloud : public Node
{
public:
    ExtractTimeStampCloud();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    Input* input_;
    Output* output_;
    Output* output_frame_;
};

}

#endif // EXTRACT_TIMESTAMP_CLOUD_H_
