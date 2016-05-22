/// HEADER
#include "segmentation_ellipsoid.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <utils_laser_processing/segmentation/ellipsoid.h>

using namespace csapex;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::SegmentationEllipsoid, csapex::Node)

SegmentationEllipsoid::SegmentationEllipsoid()
{
}

void SegmentationEllipsoid::setup(NodeModifier& node_modifier)
{
    ScanSegmentation::setup(node_modifier);
    update();
}

void SegmentationEllipsoid::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("support points",
                                                                  1, 10, 1, 1),
                            std::bind(&SegmentationEllipsoid::update, this));
    parameters.addParameter(param::ParameterFactory::declareRange("support point distance",
                                                                  0.01, 0.2, 0.05, 0.01),
                            std::bind(&SegmentationEllipsoid::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareInterval("radius",
                                                                  0.05, 0.3, 0.15, 0.25, 0.01),
                            std::bind(&SegmentationEllipsoid::update, this));

}

void SegmentationEllipsoid::update()
{
    double support_points = readParameter<int>("support points");
    double support_point_distance = readParameter<double>("support point distance");
    std::pair<double, double> radius = readParameter<std::pair<double,double>>("radius");

    segmentation_.reset(new Ellipsoid(support_points,
                                      support_point_distance,
                                      radius.first,
                                      radius.second));
}

