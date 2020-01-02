#ifndef EXTRACT_TIMESTAMP_CLOUD_H_
#define EXTRACT_TIMESTAMP_CLOUD_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class ExtractTimeStampCloud : public Node
{
public:
    ExtractTimeStampCloud();

    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_;
    Output* output_;
    Output* output_frame_;
};

}  // namespace csapex

#endif  // EXTRACT_TIMESTAMP_CLOUD_H_
