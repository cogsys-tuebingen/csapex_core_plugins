#include "segmentation_linefit.h"

#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <cslibs_laser_processing/segmentation/line_fit.h>

using namespace csapex;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::LineFitSegmentation, csapex::Node)

LineFitSegmentation::LineFitSegmentation()
{
}

void LineFitSegmentation::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("sigma", 0.01, 2.0, 0.01, 0.01), std::bind(&LineFitSegmentation::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("max. distance", 0.01, 2.0, 0.01, 0.01), std::bind(&LineFitSegmentation::update, this));
    //    parameters.addParameter(csapex::param::factory::declareRange("segment
    //    lines"), false);
}

void LineFitSegmentation::update()
{
    double sigma = readParameter<double>("sigma");
    double max_dist = readParameter<double>("max. distance");
    segmentation_.reset(new LineFit(sigma, max_dist));
}

void LineFitSegmentation::setup(NodeModifier& node_modifier)
{
    ScanSegmentation::setup(node_modifier);
    update();
}
