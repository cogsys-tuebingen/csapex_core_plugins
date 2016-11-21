#ifndef VECTORPLOT_H
#define VECTORPLOT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/assert.h>
#include <csapex/model/variadic_io.h>

/// SYSTEM
#include <chrono>
#include <QColor>
#include <qwt_plot_curve.h>
#include <qwt_plot_scaleitem.h>
#include <qwt_scale_map.h>
namespace csapex{
class VectorPlot : public csapex::Node, public csapex::VariadicInputs
{
public:
    VectorPlot();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override;

    int getWidth() const;
    int getHeight() const;
    double getLineWidth() const;

    QColor getBackgroundColor() const;
    QColor getLineColor(std::size_t idx) const;
    QColor getFillColor() const;

    const double *getTData() const;
    const double* getVData(std::size_t idx) const;
    std::size_t getVDataCountNumCurves() const;
    std::size_t getCount() const;

    const QwtScaleMap& getXMap() const;
    const QwtScaleMap& getYMap() const;

public:


    slim_signal::Signal<void()> update;
    slim_signal::Signal<void()> display_request;

protected:
    void reset();
    void init();

    void preparePlot();
    void renderAndSend();

private:
    mutable std::recursive_mutex mutex_;
    csapex::Input* in_time_;
    csapex::Output* out_;

    bool initialize_;
    bool basic_line_color_changed_;
    bool has_time_in_;
    std::size_t num_plots_;
    Input* in_;


    int width_;
    int height_;

    QColor color_bg_;
    QColor basic_line_color_;
    std::vector<QColor> color_line_;
    QColor color_fill_;

    double line_width_;

    bool time_relative_;
    bool time_seconds_;

    double start_t_;
    std::vector<double> data_t_raw_;


    std::vector<double> data_t_;
    std::vector<std::vector<double>> data_v_;

    QwtScaleMap x_map;
    QwtScaleMap y_map;

private:
    void calculateLineColors();
};

}
#endif // VECTORPLOT_H
