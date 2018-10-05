#include "segmentation_segment_length.h"

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <cslibs_laser_processing/segmentation/segment_length.h>

using namespace csapex;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::SegmentLengthSegmentation, csapex::Node)

SegmentLengthSegmentation::SegmentLengthSegmentation()
{
}

void SegmentLengthSegmentation::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("max distance", 0.01, 2.0, 0.01, 0.01), std::bind(&SegmentLengthSegmentation::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("max length", 0.01, 2.0, 0.01, 0.01), std::bind(&SegmentLengthSegmentation::update, this));
}

void SegmentLengthSegmentation::update()
{
    double max_dist = readParameter<double>("max distance");
    double max_length = readParameter<double>("max length");
    segmentation_.reset(new SegmentLength(max_dist, max_length));
}

void SegmentLengthSegmentation::setup(NodeModifier& node_modifier)
{
    ScanSegmentation::setup(node_modifier);
    update();
}
