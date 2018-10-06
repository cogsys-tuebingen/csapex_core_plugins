#ifndef CROP_BOX_H_
#define CROP_BOX_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class CropBox : public Node
{
public:
    CropBox();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_cloud_;
    Output* output_pos_;
    Output* output_neg_;
};

}  // namespace csapex

#endif  // CROP_BOX_H_
