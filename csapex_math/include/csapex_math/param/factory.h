#ifndef FACTORY_H
#define FACTORY_H

/// PROJECT
#include <csapex/param/parameter_builder.h>

/// COMPONENT
#include <csapex_math/math_fwd.h>

namespace csapex
{
namespace param
{
namespace factory
{
/**
 * @brief declareAngleParameter
 * @param name
 * @param description
 * @param angle default value
 * @return
 */
ParameterBuilder declareAngle(const std::string& name, const ParameterDescription& description, double angle);
ParameterBuilder declareAngle(const std::string& name, double angle);

ParameterBuilder declareVector(const std::string& name, const std::vector<double>& vector);
ParameterBuilder declareVector(const std::string& name, const ParameterDescription& description, const std::vector<double>& vector);

ParameterBuilder declareVector(const std::string& name, const math::linear::Vector& vector);
ParameterBuilder declareVector(const std::string& name, const ParameterDescription& description, const math::linear::Vector& vector);

ParameterBuilder declareMatrix(const std::string& name, const math::linear::Matrix& matrix);
ParameterBuilder declareMatrix(const std::string& name, const ParameterDescription& description, const math::linear::Matrix& matrix);

}  // namespace factory
}  // namespace param
}  // namespace csapex

#endif  // FACTORY_H
