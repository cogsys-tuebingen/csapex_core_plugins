#ifndef POINT_COUNT_H
#define POINT_COUNT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {

class TransformCloud : public Node
{
public:
    TransformCloud();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_cloud_;
    Input* input_transform_;
    Output* output_;
};

}

#endif // POINT_COUNT_H
