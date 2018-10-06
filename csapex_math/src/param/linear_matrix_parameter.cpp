/// HEADER
#include <csapex_math/param/linear_matrix_parameter.h>

/// COMPONENT
#include <csapex_math/serialization/binary_io.h>
#include <csapex_math/serialization/yaml_io.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <boost/any.hpp>
#include <yaml-cpp/yaml.h>

CSAPEX_REGISTER_PARAM(LinearMatrixParameter)

using namespace csapex;
using namespace param;

LinearMatrixParameter::LinearMatrixParameter(const std::string& name, const ParameterDescription& description, math::linear::Matrix value) : ParameterTemplate(name, description, value)
{
}
