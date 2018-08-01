#include "segmentation_linefit_lsq.h"

#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <cslibs_laser_processing/segmentation/line_fit_lsq.h>

using namespace csapex;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::LineFitSegmentationLSQ, csapex::Node)

LineFitSegmentationLSQ::LineFitSegmentationLSQ()
{
}

void LineFitSegmentationLSQ::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("delta_d",    0.01, 2.0, 0.01, 0.01),
                            std::bind(&LineFitSegmentationLSQ::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("delta_var",  0.01, 2.0, 0.01, 0.01),
                            std::bind(&LineFitSegmentationLSQ::update, this));
}

void LineFitSegmentationLSQ::update()
{
    double delta_d    = readParameter<double>("delta_d");
    double delta_var  = readParameter<double>("delta_var");
    segmentation_.reset(new LineFitLSQ(delta_d, delta_var));
}

void LineFitSegmentationLSQ::setup(NodeModifier& node_modifier)
{
    ScanSegmentation::setup(node_modifier);
    update();
}
