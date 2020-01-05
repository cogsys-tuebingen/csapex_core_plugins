/// HEADER
#include <csapex_math/param/linear_vector_parameter.h>

/// COMPONENT
#include <csapex_math/serialization/binary_io.h>
#include <csapex_math/serialization/yaml_io.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/param/value_parameter.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <boost/any.hpp>

CSAPEX_REGISTER_PARAM(LinearVectorParameter)

using namespace csapex;
using namespace param;

LinearVectorParameter::LinearVectorParameter(const std::string& name, const ParameterDescription& description, math::linear::Vector value) : ParameterTemplate(name, description, value)
{
}
