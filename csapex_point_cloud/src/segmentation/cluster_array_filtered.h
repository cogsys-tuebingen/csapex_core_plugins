#ifndef CLUSTERPOINTCLOUDARRAY_H
#define CLUSTERPOINTCLOUDARRAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

#include "regular_structures/cluster_params.hpp"

namespace csapex {
class ClusterPointCloudArrayFiltered : public csapex::Node
{
public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input*        in_cloud_;
    Input*        in_indices_;
    Output*       out_;
    Output*       out_rejected_;

    ClusterParams cluster_params_;

};
}

#endif // CLUSTERPOINTCLOUDARRAY_H
