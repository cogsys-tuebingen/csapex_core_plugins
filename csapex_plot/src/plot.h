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

protected:
    Plot();

    std::function<void (param::Parameter *)> setColor(QColor& p);

public:
    slim_signal::Signal<void()> update;
    slim_signal::Signal<void()> display_request;



protected:
    QwtScaleMap x_map;
    QwtScaleMap y_map;

    int width_;
    int height_;

    QColor color_bg_;
    QColor color_fill_;
};

}
#endif // Plot_H
