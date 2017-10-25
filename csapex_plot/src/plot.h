#ifndef PLOT_H
#define PLOT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <chrono>
#include <QColor>
#include <qwt_plot_curve.h>
#include <qwt_plot_scaleitem.h>
#include <qwt_scale_map.h>
#include <functional>

namespace csapex{

class Plot : public csapex::Node
{
public:
    virtual ~Plot();

    void setupParameters(Parameterizable &parameters);

    const QwtScaleMap& getXMap() const;
    const QwtScaleMap& getYMap() const;

    int getWidth() const;
    int getHeight() const;

    QColor getBackgroundColor() const;
    QColor getFillColor() const;
    QColor getLineColor(std::size_t idx) const;

    std::size_t getNumberOfPlots() const;
    void updateLineColors();

protected:
    Plot();

    std::function<void (param::Parameter *)> setColor(QColor& p);

public:
    slim_signal::Signal<void()> update;
    slim_signal::Signal<void()> display_request;



protected:
    mutable std::recursive_mutex mutex_;
    QwtScaleMap x_map;
    QwtScaleMap y_map;

    bool basic_line_color_changed_;
    int width_;
    int height_;
    std::size_t num_plots_;
    std::size_t last_num_plots_;

    QColor color_bg_;
    QColor color_fill_;
    QColor basic_line_color_;
    std::vector<QColor> color_line_;
};

}
#endif // Plot_H
