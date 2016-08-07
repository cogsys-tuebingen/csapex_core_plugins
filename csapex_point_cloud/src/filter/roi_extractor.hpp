#pragma once

#include <csapex/model/node.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex
{
class ROIExtractor : public Node
{
public:
    ROIExtractor();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    template <class PointT> void extract_organized(typename pcl::PointCloud<PointT>::ConstPtr cloud);
    template <class PointT> void extract_unorganized(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    void updateOutputs();
    void publish(const std::shared_ptr< std::vector<connection_types::PointCloudMessage::Ptr> > message);

private:
    Input* input_cloud_;
    Input* input_rois_;
    Input* input_indices_;
    Output* output_clouds_;
    bool filter_;
    int filter_class_;
    std::vector<Output*> output_clouds_single_;
};
}
