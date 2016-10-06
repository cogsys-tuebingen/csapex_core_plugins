#ifndef TIME_PLOT_H
#define TIME_PLOT_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <chrono>
#include <QColor>
#include <qwt_plot_curve.h>
#include <qwt_plot_scaleitem.h>
#include <qwt_scale_map.h>
#include <deque>

namespace csapex
{

class TimePlot : public Node
{
private:
    typedef std::chrono::system_clock::time_point timepoint;

public:
    void setup(NodeModifier &node_modifier);

    void setupParameters(Parameterizable &parameters);

    void process();

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

protected:
    void reset();
    void init();

    void preparePlot();
    void renderAndSend();

public:
    csapex::slim_signal::Signal<void()> update;
    csapex::slim_signal::Signal<void()> display_request;

private:
    bool initialize_;
    bool basic_line_color_changed_;
    std::size_t num_plots_;
    std::size_t deque_size_;
    Input* in_;
    Output* out_;

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
    std::deque<double> data_t_raw_;

    std::vector<double> data_t_;
    std::vector<std::deque<double>> deque_v_;
    std::vector<std::vector<double>> data_v_;

    QwtScaleMap x_map;
    QwtScaleMap y_map;

private:
    void calculateLineColors();
};

}

#endif // TIME_PLOT_H
