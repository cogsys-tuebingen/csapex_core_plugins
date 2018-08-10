#ifndef VECTORPLOT_H
#define VECTORPLOT_H

/// PROJECT
#include "plot.h"
#include <csapex/utility/assert.h>
#include <csapex/model/variadic_io.h>

/// SYSTEM
#include <chrono>
#include <QColor>
#include <qwt_plot_curve.h>
#include <qwt_plot_scaleitem.h>
#include <qwt_scale_map.h>

namespace csapex{
class VectorPlot : public Plot, public csapex::VariadicInputs
{
public:
    VectorPlot();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override;

    double getLineWidth() const;

//    QColor getLineColor(std::size_t idx) const;

    const double *getTData() const;
    const double* getVData(std::size_t idx) const;
    std::size_t getVDataCountNumCurves() const;
    std::size_t getCount() const;

protected:
    void reset();
    void init();

    void preparePlot();
    void renderAndSend();

private:
    mutable std::recursive_mutex mutex_buffer_;
    csapex::Input* in_time_;
    csapex::Output* out_;

    bool initialize_;
    bool basic_line_color_changed_;
    bool has_time_in_;
    Input* in_;

    double line_width_;

    bool time_relative_;
    bool time_seconds_;

    double start_t_;
    std::vector<double> data_t_raw_;


    std::vector<double> data_t_;
    std::vector<std::vector<double>> data_v_;

};

}
#endif // VECTORPLOT_H
