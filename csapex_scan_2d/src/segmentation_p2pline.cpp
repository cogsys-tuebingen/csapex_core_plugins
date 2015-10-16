/// HEADER
#include "segmentation_p2pline.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <utils_laser_processing/segmentation/p2pline.h>

using namespace csapex;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::P2PLineSegmentation, csapex::Node)

P2PLineSegmentation::P2PLineSegmentation()
{
}

void P2PLineSegmentation::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("sigma",          0.01, 2.0, 0.01, 0.01), std::bind(&P2PLineSegmentation::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("max. distance",  0.01, 2.0, 0.01, 0.01), std::bind(&P2PLineSegmentation::update, this));
}

void P2PLineSegmentation::update()
{
    double sigma    = readParameter<double>("sigma");
    double max_dist = readParameter<double>("max. distance");
    segmentation_.reset(new P2PLine(sigma, max_dist));
}

void P2PLineSegmentation::setup(NodeModifier& node_modifier)
{
    ScanSegmentation::setup(node_modifier);
    update();
}
