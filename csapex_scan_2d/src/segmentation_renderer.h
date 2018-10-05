#ifndef SCAN_SEGMENTATION_RENDERER_H
#define SCAN_SEGMENTATION_RENDERER_H

/// PROJECT
#include <csapex/model/node.h>
#include <cslibs_laser_processing/data/segment.h>

/// SYSTEM
#include <geometry_msgs/Point.h>

namespace csapex
{
class ScanSegmentation2DRenderer : public csapex::Node
{
public:
    ScanSegmentation2DRenderer();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

private:
    void publishMarkers(const std::vector<lib_laser_processing::Segment>& segments);
    void render(const std::vector<lib_laser_processing::Segment>& segments);

    geometry_msgs::Point toGeometry(const Eigen::Vector2d& vec);

private:
    /// APEX
    Input* input_;
    Output* output_;
    Output* output_marker_;

    /// RENDERING
    int last_id_;
};
}  // namespace csapex
#endif  // SCAN_SEGMENTATION_RENDERER_H
