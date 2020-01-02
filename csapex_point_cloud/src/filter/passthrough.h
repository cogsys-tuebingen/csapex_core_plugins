#ifndef PASSTHROUGH_H_
#define PASSTHROUGH_H_

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class PassThrough : public Node
{
public:
    PassThrough();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    void updateBorders();
    void updateFields(const std::vector<std::string>& fields);

private:
    Input* input_cloud_;
    Output* output_pos_;
    Output* output_neg_;

    std::vector<std::string> fields_;
};

}  // namespace csapex

#endif  // PASSTHROUGH_H_
