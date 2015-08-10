#ifndef MERGE_CLOUDS_H
#define MERGE_CLOUDS_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM

namespace csapex {


class MergeClouds : public csapex::Node
{
public:
    MergeClouds();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    void updateInputs();

private:
    Output* out_;

    connection_types::PointCloudMessage::Ptr result_;
};


}

#endif // MERGE_CLOUDS_H
