#pragma once

#include <csapex/model/node.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace csapex
{
class EstimateCenter : public Node
{
public:
    EstimateCenter();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_clouds_;
    Output* output_poses_;
    Output* output_poses_covariance_;

    bool param_use_bounding_box_;
    bool param_assume_half_visible_;

    struct
    {
        std::string frame_id;
        csapex::connection_types::Message::Stamp stamp_micro_seconds;
    } tmp;

    std::vector<std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>> output_;
};
}  // namespace csapex
