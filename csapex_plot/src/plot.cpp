/// HEADER
#include "plot.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex/view/utility/color.hpp>
#include <csapex_opencv/cv_mat_message.h>
/// SYSTEM
#include <libqwt/qwt_scale_engine.h>

using namespace csapex;
using namespace csapex::connection_types;

Plot::Plot() : basic_line_color_changed_(true), width_(640), height_(320), num_plots_(1), last_num_plots_(0)
{
}

Plot::~Plot()
{
}

void Plot::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareColorParameter("~plot/color/background", 255, 255, 255), setColor(color_bg_));
    parameters.addParameter(param::factory::declareColorParameter("~plot/color/fill", 200, 200, 200), setColor(color_fill_));

    parameters.addParameter(param::factory::declareColorParameter("~plot/color/line", 100, 100, 255), [this](param::Parameter* p) {
        std::vector<int> c = p->as<std::vector<int>>();
        basic_line_color_.setRed(c[0]);
        basic_line_color_.setGreen(c[1]);
        basic_line_color_.setBlue(c[2]);
        updateLineColors();
        basic_line_color_changed_ = true;
        update();
    });
}

int Plot::getWidth() const
{
    return width_;
}

int Plot::getHeight() const
{
    return height_;
}

const QwtScaleMap& Plot::getXMap() const
{
    return x_map;
}
const QwtScaleMap& Plot::getYMap() const
{
    return y_map;
}

QColor Plot::getBackgroundColor() const
{
    return color_bg_;
}
QColor Plot::getFillColor() const
{
    return color_fill_;
}

std::function<void(param::Parameter*)> Plot::setColor(QColor& color)
{
    return [&](param::Parameter* p) {
        std::vector<int> c = p->as<std::vector<int>>();
        color.setRed(c[0]);
        color.setGreen(c[1]);
        color.setBlue(c[2]);

        update();
    };
}

QColor Plot::getLineColor(std::size_t idx) const
{
    return color_line_.at(idx);
}

std::size_t Plot::getNumberOfPlots() const
{
    return num_plots_;
}

void Plot::updateLineColors()
{
    //    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if (!basic_line_color_changed_ && num_plots_ == last_num_plots_) {
        return;
    }

    color_line_.resize(num_plots_);
    int r, g, b, a;
    basic_line_color_.getRgb(&r, &g, &b, &a);
    QColor color;
    color.setRed(r);
    color.setGreen(g);
    color.setBlue(b);
    int h, s, v;
    color.getHsv(&h, &s, &v);

    for (std::size_t i = 0; i < num_plots_; ++i) {
        double hnew = h + i * 360 / num_plots_;
        while (hnew > 359 || hnew < 0) {
            if (hnew > 359) {
                hnew -= 360;
            } else if (hnew < 0) {
                hnew += 360;
            }
        }
        color.setHsv(hnew, s, v);
        color_line_[i] = color;
    }

    basic_line_color_changed_ = false;
    last_num_plots_ = num_plots_;
    update();
}
