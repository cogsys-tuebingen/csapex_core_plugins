#ifndef POINT_COUNT_H
#define POINT_COUNT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
class PointCount : public Node
{
public:
    PointCount();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& params) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input* input_;
    Output* output_;

    bool filter_;

public:
    slim_signal::Signal<void(int)> display_request;
};

}  // namespace csapex

#endif  // POINT_COUNT_H
