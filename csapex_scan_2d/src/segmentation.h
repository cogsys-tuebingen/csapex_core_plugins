#ifndef SCAN_SEGMENTATION_H
#define SCAN_SEGMENTATION_H

/// COMPONENT
#include <utils_laser_processing/segmentation/segmentation.h>

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class ScanSegmentation : public csapex::Node
{
public:
    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

protected:
    ScanSegmentation();

    /// APEX
    Input            *input_;
    Output           *output_scan_;
    Output           *output_segments_;

    /// ALGORITHM
    lib_laser_processing::LaserScanSegmentation::Ptr segmentation_;
};
}
#endif // SCAN_SEGMENTATION_H
