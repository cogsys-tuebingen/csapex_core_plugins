#ifndef LINEAR_MATRIX_PARAMETER_H
#define LINEAR_MATRIX_PARAMETER_H

/// COMPONENT
#include <csapex/param/parameter_template.hpp>
#include <csapex_math/model/matrix.h>
#include <csapex_math/serialization/binary_io.h>
#include <csapex_math/serialization/stream_io.h>
#include <csapex_math/serialization/yaml_io.h>
#include <csapex_param_export.h>

/// SYSTEM
#define _USE_MATH_DEFINES
#include <math.h>

namespace csapex
{
namespace param
{
class CSAPEX_PARAM_EXPORT LinearMatrixParameter : public ParameterTemplate<math::linear::Matrix, LinearMatrixParameter>
{
public:
    typedef std::shared_ptr<LinearMatrixParameter> Ptr;

public:
    LinearMatrixParameter() = default;
    explicit LinearMatrixParameter(const std::string& name, const ParameterDescription& description, math::linear::Matrix value);
};

template <>
CSAPEX_MATH_EXPORT inline std::string serializationName<LinearMatrixParameter>()
{
    return "linear matrix";
}

}  // namespace param
}  // namespace csapex

#endif  // LINEAR_MATRIX_PARAMETER_H
