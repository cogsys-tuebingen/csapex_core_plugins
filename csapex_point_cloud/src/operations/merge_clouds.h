#ifndef MERGE_CLOUDS_H
#define MERGE_CLOUDS_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/variadic_io.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

/// SYSTEM

namespace csapex
{
class MergeClouds : public csapex::Node, public VariadicInputs
{
public:
    MergeClouds();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override;

private:
    void updateInputs();

private:
    Output* out_;

    connection_types::PointCloudMessage::Ptr result_;
};

}  // namespace csapex

#endif  // MERGE_CLOUDS_H
