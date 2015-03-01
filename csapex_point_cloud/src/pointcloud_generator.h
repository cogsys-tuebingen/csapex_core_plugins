#ifndef POINTCLOUD_GENERATOR_H
#define POINTCLOUD_GENERATOR_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class PointCloudGenerator : public csapex::Node
{
public:
    PointCloudGenerator();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

private:
    Output *out_;
};
}

#endif // POINTCLOUD_GENERATOR_H
