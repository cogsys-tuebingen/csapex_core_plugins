/// HEADER
#include "time_plot.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/view/utility/QtCvImageConverter.h>

/// SYSTEM
#include <qwt_scale_engine.h>

CSAPEX_REGISTER_CLASS(csapex::TimePlot, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


void TimePlot::setup(NodeModifier &node_modifier)
{
    in_  = node_modifier.addInput<double>("Double");
    out_ = node_modifier.addOutput<CvMatMessage>("Plot");
}

void TimePlot::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange
                            ("~plot/width",
                             128, 4096, 640, 1),
                            [this](param::Parameter* p) {
        width_ = p->as<int>();
        update();
    });
    parameters.addParameter(param::ParameterFactory::declareRange
                            ("~plot/height",
                             128, 4096, 320, 1),
                            [this](param::Parameter* p) {
        height_ = p->as<int>();
        update();
    });

    auto setColor = [this](QColor& color) {
        return [&](param::Parameter* p) {
            std::vector<int> c = p->as<std::vector<int>>();
            color.setRed(c[0]);
            color.setGreen(c[1]);
            color.setBlue(c[2]);

            update();
        };
    };

    parameters.addParameter(param::ParameterFactory::declareColorParameter(
                                "~plot/color/background", 255, 255, 255),
                            setColor(color_bg_));
    parameters.addParameter(param::ParameterFactory::declareColorParameter(
                                "~plot/color/line", 100, 100, 255),
                            setColor(color_line_));
    parameters.addParameter(param::ParameterFactory::declareColorParameter(
                                "~plot/color/fill", 200, 200, 200),
                            setColor(color_fill_));
    parameters.addParameter(param::ParameterFactory::declareRange(
                                "~plot/line/width", 0.0, 10.0, 0.0, 0.01),
                            line_width_);

    parameters.addParameter(param::ParameterFactory::declareBool
                            ("~output/time/relative",
                             param::ParameterDescription("Use time relative to the first entry"),
                             true),
                            time_relative_);
    parameters.addParameter(param::ParameterFactory::declareBool
                            ("~output/time/seconds",
                             param::ParameterDescription("Convert the time to seconds"),
                             true),
                            time_seconds_);

    parameters.addParameter(param::ParameterFactory::declareTrigger("reset"),
                            [this](param::Parameter*) {
        reset();
    });
}

void TimePlot::process()
{
    timepoint time = std::chrono::system_clock::now();
    double value = msg::getValue<double>(in_);

    double ms = std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch()).count();

    data_t_raw_.push_back(ms);
    data_v_.push_back(value);

    preparePlot();

    if(msg::isConnected(out_)) {
        renderAndSend();
    }

    display_request();
}

void TimePlot::reset()
{
    data_t_raw_.clear();
    data_v_.clear();

    init();
}

void TimePlot::init()
{
    auto now = std::chrono::system_clock::now();
    start_t_ = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

void TimePlot::preparePlot()
{
    std::size_t n = data_t_raw_.size();
    data_t_.resize(n, 0.0);

    double time_offset = time_relative_ ? data_t_raw_.front() : 0.0;
    double time_scale = time_seconds_ ? 1e-6 : 1.0;

    for(std::size_t i = 0; i < n; ++i) {
        data_t_[i] = (data_t_raw_[i] - time_offset) * time_scale;
    }


    double min = std::min(0.0, *std::min_element(data_v_.begin(), data_v_.end()));
    double max = std::max(0.0, *std::max_element(data_v_.begin(), data_v_.end()));

    x_map.setScaleInterval(data_t_.front(), data_t_.back());
    y_map.setScaleInterval(min - 1, max + 1);
}

void TimePlot::renderAndSend()
{
    QImage image(width_, height_, QImage::Format_RGB32);
    QPainter painter;
    painter.begin(&image);
    painter.fillRect(image.rect(), color_bg_);

    QRect r(0, 0, image.width(), image.height());

    x_map.setPaintInterval( r.left(), r.right() );
    y_map.setPaintInterval( r.bottom(), r.top() );


    QwtPlotCurve curve;
    painter.setRenderHint( QPainter::Antialiasing,
                           curve.testRenderHint( QwtPlotItem::RenderAntialiased ) );
    painter.setRenderHint( QPainter::HighQualityAntialiasing,
                           curve.testRenderHint( QwtPlotItem::RenderAntialiased ) );
    curve.setBaseline(0.0);

    curve.setPen(color_line_, line_width_);
    curve.setStyle(QwtPlotCurve::Lines);

    curve.setBrush(QBrush(color_fill_, Qt::SolidPattern));

    curve.setRawSamples(data_t_.data(), data_v_.data(), data_t_.size());
    curve.draw(&painter, x_map, y_map, r);


    QwtLinearScaleEngine e;
    QwtPlotScaleItem scale_time(QwtScaleDraw::TopScale, y_map.s1());
    scale_time.setScaleDiv(e.divideScale(x_map.s1(), x_map.s2(), 10, 10));

    QwtPlotScaleItem scale_value(QwtScaleDraw::RightScale, x_map.s1());
    scale_value.setScaleDiv(e.divideScale(y_map.s1(), y_map.s2(), 10, 10));

    scale_time.draw(&painter, x_map, y_map, r);
    scale_value.draw(&painter, x_map, y_map, r);


    CvMatMessage::Ptr out_msg = std::make_shared<CvMatMessage>(enc::bgr, 0);
    out_msg->value = QtCvImageConverter::Converter<QImage>::QImage2Mat(image);

    msg::publish(out_, out_msg);
}

int TimePlot::getWidth() const
{
    return width_;
}

int TimePlot::getHeight() const
{
    return height_;
}

double TimePlot::getLineWidth() const
{
    return line_width_;
}

QColor TimePlot::getBackgroundColor() const
{
    return color_bg_;
}
QColor TimePlot::getLineColor() const
{
    return color_line_;
}
QColor TimePlot::getFillColor() const
{
    return color_fill_;
}

const double* TimePlot::getTData() const
{
    return data_t_.data();
}
const double* TimePlot::getVData() const
{
    return data_v_.data();
}
std::size_t TimePlot::getCount() const
{
    return data_t_.size();
}

const QwtScaleMap& TimePlot::getXMap() const
{
    return x_map;
}
const QwtScaleMap& TimePlot::getYMap() const
{
    return y_map;
}
