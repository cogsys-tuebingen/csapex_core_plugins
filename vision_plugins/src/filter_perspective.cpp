/// HEADER
#include "filter_perspective.h"

/// PROJECT
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(vision_plugins::PerspectiveTransform, csapex::Node)

using namespace vision_plugins;
using namespace csapex;

PerspectiveTransform::PerspectiveTransform()
{
    addParameter(param::ParameterFactory::declareRange("Rotation x", -90.0, 90.0, 0.0, 0.1), boost::bind(&PerspectiveTransform::update, this));
    addParameter(param::ParameterFactory::declareRange("Rotation y",  -90.0, 90.0, 0.0, 0.1), boost::bind(&PerspectiveTransform::update, this));
    addParameter(param::ParameterFactory::declareRange("Rotation z",  -90.0, 90.0, 0.0, 0.1), boost::bind(&PerspectiveTransform::update, this));
    addParameter(param::ParameterFactory::declareRange("Virt. Distance", 0.0, 2000.0, 500.0, 0.1), boost::bind(&PerspectiveTransform::update, this));
    addParameter(param::ParameterFactory::declareRange("Virt. Focal Length", 0.0, 2000.0, 500.0, 0.1), boost::bind(&PerspectiveTransform::update, this));
}

void PerspectiveTransform::filter(cv::Mat &img, cv::Mat &mask)
{
    /// mask is unused
    transformer_.transform(img, img);
}

void PerspectiveTransform::update()
{
    double rot_x = readParameter<double>("Rotation x");
    double rot_y = readParameter<double>("Rotation y");
    double rot_z = readParameter<double>("Rotation z");
    double foca  = readParameter<double>("Virt. Distance");
    double dist  = readParameter<double>("Virt. Focal Length");
    transformer_.set_rot_x(rot_x);
    transformer_.set_rot_y(rot_y);
    transformer_.set_rot_z(rot_z);
    transformer_.set_distance(dist);
    transformer_.set_focal(foca);
}



