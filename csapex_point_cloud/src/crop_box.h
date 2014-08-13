#ifndef CROP_BOX_H_
#define CROP_BOX_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class CropBox : public Node
{
public:
    CropBox();

    virtual void setup();
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

private:
    Input* input_cloud_;
    Output* output_pos_;
    Output* output_neg_;
};

}

#endif // CROP_BOX_H_
