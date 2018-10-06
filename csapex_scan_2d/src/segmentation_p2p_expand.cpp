#include "segmentation_p2p_expand.h"

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <cslibs_laser_processing/segmentation/p2pdistance_expand.h>

using namespace csapex;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::P2PSegmentationExpand, csapex::Node)

P2PSegmentationExpand::P2PSegmentationExpand()
{
}

void P2PSegmentationExpand::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("max. distance", 0.01, 2.0, 0.01, 0.01), std::bind(&P2PSegmentationExpand::update, this));
}

void P2PSegmentationExpand::update()
{
    double max_dist = readParameter<double>("max. distance");
    segmentation_.reset(new P2PDistanceExpand(max_dist));
}

void P2PSegmentationExpand::setup(NodeModifier& node_modifier)
{
    ScanSegmentation::setup(node_modifier);
    update();
}
