#include "segmentation_p2p.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <utils_laser_processing/segmentation/p2pdistance.h>

using namespace csapex;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::P2PSegmentation, csapex::Node)

P2PSegmentation::P2PSegmentation()
{

}

void P2PSegmentation::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("max. distance",  0.01, 2.0, 0.01, 0.01), std::bind(&P2PSegmentation::update, this));
}

void P2PSegmentation::update()
{
    double max_dist = readParameter<double>("max. distance");
    segmentation_.reset(new P2PDistance(max_dist));
}

void P2PSegmentation::setup(NodeModifier& node_modifier)
{
    ScanSegmentation::setup(node_modifier);
    update();
}
