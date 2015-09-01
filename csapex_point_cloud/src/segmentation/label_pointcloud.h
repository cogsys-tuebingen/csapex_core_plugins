#ifndef LABEL_POINTCLOUD_H
#define LABEL_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_vision/cv_mat_message.h>

namespace csapex {
class LabelPointCloud : public csapex::Node
{
public:
    LabelPointCloud();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input*  input_;
    Input*  labels_;
    Output* output_;
    connection_types::CvMatMessage::ConstPtr label_msg_;
};
}
#endif // LABEL_POINTCLOUD_H
