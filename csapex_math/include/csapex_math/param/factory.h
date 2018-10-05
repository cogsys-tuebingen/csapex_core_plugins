#ifndef FACTORY_H
#define FACTORY_H

/// PROJECT
#include <csapex/param/parameter_builder.h>

/// COMPONENT
#include <csapex_math/param/linear_vector_parameter.h>

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

}  // namespace factory
}  // namespace param
}  // namespace csapex

#endif  // FACTORY_H
