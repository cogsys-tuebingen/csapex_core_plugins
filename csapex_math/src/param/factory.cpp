/// HEADER
#include <csapex_math/param/factory.h>

/// COMPONENT
#include <csapex_math/param/angle_parameter.h>

using namespace csapex;
using namespace csapex::param;

ParameterBuilder factory::declareAngle(const std::string& name, const ParameterDescription& description, double angle)
{
    std::shared_ptr<AngleParameter> result(new AngleParameter(name, description, angle));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareAngle(const std::string& name, double angle)
{
    return declareAngle(name, ParameterDescription(), angle);
}
