/// HEADER
#include <csapex_math/param/factory.h>

/// COMPONENT
#include <csapex_math/param/angle_parameter.h>
#include <csapex_math/param/linear_vector_parameter.h>
#include <csapex_math/param/linear_matrix_parameter.h>

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

ParameterBuilder factory::declareVector(const std::string& name, const std::vector<double>& vector)
{
    return declareVector(name, ParameterDescription(), vector);
}

ParameterBuilder factory::declareVector(const std::string& name, const ParameterDescription& description, const std::vector<double>& vector)
{
    return declareVector(name, description, math::linear::Vector(vector));
}

ParameterBuilder factory::declareVector(const std::string& name, const math::linear::Vector& vector)
{
    return declareVector(name, ParameterDescription(), vector);
}

ParameterBuilder factory::declareVector(const std::string& name, const ParameterDescription& description, const math::linear::Vector& vector)
{
    std::shared_ptr<LinearVectorParameter> result(new LinearVectorParameter(name, description, vector));
    return ParameterBuilder(std::move(result));
}

ParameterBuilder factory::declareMatrix(const std::string& name, const math::linear::Matrix& matrix)
{
    return declareMatrix(name, ParameterDescription(), matrix);
}

ParameterBuilder factory::declareMatrix(const std::string& name, const ParameterDescription& description, const math::linear::Matrix& matrix)
{
    std::shared_ptr<LinearMatrixParameter> result(new LinearMatrixParameter(name, description, matrix));
    return ParameterBuilder(std::move(result));
}
