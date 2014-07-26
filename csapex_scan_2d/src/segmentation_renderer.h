#ifndef SCAN_SEGMENTATION_RENDERER_H
#define SCAN_SEGMENTATION_RENDERER_H

/// PROJECT
#include <csapex/model/node.h>
#include <utils_laser_processing/data/segment.h>

/// SYSTEM
#include <geometry_msgs/Point.h>

namespace csapex {
class ScanSegmentation2DRenderer : public csapex::Node
{
public:
    ScanSegmentation2DRenderer();

    virtual void process();
    virtual void setup();

private:
    void publishMarkers(const std::vector<lib_laser_processing::Segment> &segments);
    void render(const std::vector<lib_laser_processing::Segment> &segments);


    geometry_msgs::Point toGeometry(const Eigen::Vector2d &vec);

private:
    /// APEX
    Input            *input_;
    Output           *output_;
    Output           *output_marker_;

    /// RENDERING
    int                     last_id_;


};
}
#endif // SCAN_SEGMENTATION_RENDERER_H
