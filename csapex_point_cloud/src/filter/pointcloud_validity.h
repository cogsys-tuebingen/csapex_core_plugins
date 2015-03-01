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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input*  input_;
    Output* mask_;
    Output* index_;
};
}

#endif // UNDEFINED_VALUE_MASK_H
