#ifndef LABEL_POINTCLOUD_H
#define LABEL_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class LabelPointCloud : public csapex::Node
{
public:
    LabelPointCloud();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input* input_;
    Input* labels_;
    Output* output_;
    connection_types::CvMatMessage::ConstPtr label_msg_;
};
}  // namespace csapex
#endif  // LABEL_POINTCLOUD_H
