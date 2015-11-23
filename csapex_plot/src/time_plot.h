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
    QColor getLineColor() const;
    QColor getFillColor() const;

    const double *getTData() const;
    const double* getVData() const;
    std::size_t getCount() const;

    const QwtScaleMap& getXMap() const;
    const QwtScaleMap& getYMap() const;

protected:
    void reset();
    void init();

    void preparePlot();
    void renderAndSend();

public:
    boost::signals2::signal<void()> update;
    boost::signals2::signal<void()> display_request;

private:
    Input* in_;
    Output* out_;

    int width_;
    int height_;

    QColor color_bg_;
    QColor color_line_;
    QColor color_fill_;

    double line_width_;

    bool time_relative_;
    bool time_seconds_;

    double start_t_;
    std::vector<double> data_t_raw_;

    std::vector<double> data_t_;
        std::vector<double> data_v_;

    QwtScaleMap x_map;
    QwtScaleMap y_map;
};

}

#endif // TIME_PLOT_H
