#ifndef FACTORY_H
#define FACTORY_H

/// PROJECT
#include <csapex/param/parameter_builder.h>

namespace csapex {
namespace param {
namespace factory {
/**
 * @brief declareAngleParameter
 * @param name
 * @param description
 * @param angle default value
 * @return
 */
ParameterBuilder declareAngle(const std::string &name,
                              const ParameterDescription &description,
                              double angle);
ParameterBuilder declareAngle(const std::string &name, double angle);
} // namespace factory
} // namespace param
} // namespace csapex

#endif // FACTORY_H
