#ifndef OPTIMIZATION_PARAMS_H
#define OPTIMIZATION_PARAMS_H

#include <iostream>
#include <map>
#include <random>

#include <csapex/model/parameterizable.h>
namespace csapex
{
class OptimizationParams
{
public:
    OptimizationParams();

    void setupParameters(csapex::Parameterizable& params);

    std::vector<double> getRandomStart() const;
    void getBounds(std::vector<double>& lower, std::vector<double>& upper) const;

    // getter
    inline std::size_t problemDimension() const
    {
        return problem_dim_;
    }
    inline bool boundsSetted() const
    {
        return set_bounds_;
    }
    inline std::vector<double> getStartParams() const
    {
        return start_values_;
    }
    inline bool useStart() const
    {
        return set_start_;
    }
    // setter
    inline void setStartParams(const std::vector<double>& vals)
    {
        start_values_ = vals;
    }
    inline void set(csapex::Parameterizable& params)
    {
        params_ = &params;
    }
    void setDimension(std::size_t dim);
    void removeProbelmDimParam();

protected:
    void boundParams();
    void startParams();
    void removeStart(std::size_t i);
    void removeBounds(std::size_t i);
    void checkBounds();
    static void setFormExp(double& tar, param::Parameter* p);

protected:
    csapex::Parameterizable* params_;
    bool set_bounds_;
    bool relative_bounds_;
    bool one_bound_for_all_;
    bool set_start_;
    std::size_t problem_dim_;
    std::vector<double> lower_bounds_;
    std::vector<double> upper_bounds_;
    std::vector<double> start_values_;
};
}  // namespace csapex
#endif  // OPTIMIZATION_PARAMS_H
