#ifndef SCATTER_PLOT_H
#define SCATTER_PLOT_H

/// PROJECT
#include "plot.h"
#include <csapex/model/variadic_io.h>

/// SYSTEM
#include <QColor>
#include <chrono>
#include <deque>
#include <qwt_plot_curve.h>
#include <qwt_plot_scaleitem.h>
#include <qwt_scale_map.h>

namespace csapex
{
class ScatterPlot : public Plot, public csapex::VariadicInputs
{
public:
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    const double* getXData() const;
    const double* getYData() const;
    const double* getVarData(std::size_t idx) const;
    std::size_t getCount() const;
    //    std::size_t getNumberOfPlots() const;

    double getPointSize() const;

private:
    void reset() override;

private:
    Input* in_x_;
    Input* in_y_;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<std::vector<double>> var_y_;

    double min_x_;
    double min_y_;
    double max_x_;
    double max_y_;
};

}  // namespace csapex

#endif  // SCATTER_PLOT_H
