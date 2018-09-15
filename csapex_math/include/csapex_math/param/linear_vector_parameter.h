#ifndef LINEAR_VECTOR_PARAMETER_H
#define LINEAR_VECTOR_PARAMETER_H

/// COMPONENT
#include <csapex_param_export.h>
#include <csapex_math/model/vector.h>
#include <csapex_math/serialization/yaml_io.h>
#include <csapex_math/serialization/binary_io.h>
#include <csapex_math/serialization/stream_io.h>
#include <csapex/param/parameter_template.hpp>

/// SYSTEM
#define _USE_MATH_DEFINES
#include <math.h>

namespace csapex {
namespace param {
class CSAPEX_PARAM_EXPORT LinearVectorParameter
    : public ParameterTemplate<math::linear::Vector, LinearVectorParameter> {
public:
  typedef std::shared_ptr<LinearVectorParameter> Ptr;

public:
  LinearVectorParameter() = default;
  explicit LinearVectorParameter(const std::string &name,
                           const ParameterDescription &description,
                           math::linear::Vector value);
};


template <> CSAPEX_MATH_EXPORT inline std::string serializationName<LinearVectorParameter>() {
  return "linear vector";
}

} // namespace param
} // namespace csapex

#endif // LINEAR_VECTOR_PARAMETER_H
