/// HEADER
#include "time_plot.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex_opencv/cv_mat_message.h>

/// SYSTEM
#include <qwt_scale_engine.h>

CSAPEX_REGISTER_CLASS(csapex::TimePlot, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

void TimePlot::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addMultiInput<double, GenericVectorMessage>("Double");
    out_ = node_modifier.addOutput<CvMatMessage>("Plot");
    initialize_ = true;
    num_plots_ = 1;
    basic_line_color_changed_ = true;
    color_line_.resize(1);
}

void TimePlot::setupParameters(Parameterizable& parameters)
{
    Plot::setupParameters(parameters);

    parameters.addParameter(param::factory::declareRange("~plot/width", 128, 4096, 640, 1), [this](param::Parameter* p) {
        width_ = p->as<int>();
        update();
    });
    parameters.addParameter(param::factory::declareRange("~plot/height", 128, 4096, 320, 1), [this](param::Parameter* p) {
        height_ = p->as<int>();
        update();
    });

    parameters.addParameter(param::factory::declareRange("~plot/line/width", 0.0, 10.0, 0.0, 0.01), line_width_);

    parameters.addParameter(param::factory::declareBool("~output/time/relative", param::ParameterDescription("Use time relative to the first entry"), true), time_relative_);
    parameters.addParameter(param::factory::declareBool("~output/time/seconds", param::ParameterDescription("Convert the time to seconds"), true), time_seconds_);

    parameters.addParameter(param::factory::declareValue("~output/plot/number_of_points",
                                                         param::ParameterDescription("Show only the last n samples. -1 if you want to display all "
                                                                                     "samples (might be very slow)."),
                                                         1000),
                            [this](param::Parameter* p) { deque_size_ = p->as<int>(); });

    parameters.addParameter(param::factory::declareTrigger("reset"), [this](param::Parameter*) { reset(); });
}

void TimePlot::process()
{
    timepoint time = std::chrono::system_clock::now();
    double value;
    if (msg::isValue<double>(in_)) {
        std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
        value = msg::getValue<double>(in_);
        if (initialize_) {
            deque_v_.resize(1);
            data_v_.resize(1);
            num_plots_ = 1;
            initialize_ = false;
            color_line_.resize(1);
            updateLineColors();
        }
        deque_v_.at(0).push_back(value);

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
        GenericVectorMessage::ConstPtr message = msg::getMessage<GenericVectorMessage>(in_);
        apex_assert(std::dynamic_pointer_cast<GenericValueMessage<double>>(message->nestedType()));
        num_plots_ = message->nestedValueCount();

        deque_v_.resize(num_plots_);
        color_line_.resize(num_plots_);
        updateLineColors();

        for (std::size_t num_plot = 0; num_plot < num_plots_; ++num_plot) {
            auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(message->nestedValue(num_plot));
            if (pval) {
                double val = pval->value;
                deque_v_.at(num_plot).push_back(val);
            }
            //            double val = tval->
        }
    }

    double ms = std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch()).count();

    data_t_raw_.push_back(ms);

    while (data_t_raw_.size() > deque_size_) {
        data_t_raw_.pop_front();
    }

    for (std::size_t i = 0; i < num_plots_; ++i) {
        while (deque_v_[i].size() > deque_size_) {
            deque_v_[i].pop_front();
        }
    }
    data_v_.resize(num_plots_);
    for (std::size_t i = 0; i < num_plots_; ++i) {
        data_v_[i].resize(deque_v_[i].size());
        for (std::size_t j = 0; j < deque_v_[i].size(); ++j) {
            data_v_[i][j] = deque_v_[i][j];
        }
    }

    preparePlot();

    if (msg::isConnected(out_)) {
        renderAndSend();
    }

    display_request();
}

void TimePlot::reset()
{
    data_t_raw_.clear();
    data_v_.clear();
    deque_v_.clear();
    initialize_ = true;
    num_plots_ = 1;
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
    color_fill_.setAlpha(100);

    double time_offset = time_relative_ ? data_t_raw_.front() : 0.0;
    double time_scale = time_seconds_ ? 1e-6 : 1.0;

    for (std::size_t i = 0; i < n; ++i) {
        data_t_[i] = (data_t_raw_[i] - time_offset) * time_scale;
    }

    std::vector<double> min_list;
    std::vector<double> max_list;
    for (auto data : data_v_) {
        if (data.size() != 0) {
            min_list.push_back(std::min(0.0, *std::min_element(data.begin(), data.end())));
            max_list.push_back(std::max(0.0, *std::max_element(data.begin(), data.end())));
        }
    }

    double min = std::min(0.0, *std::min_element(min_list.begin(), min_list.end()));
    double max = std::max(0.0, *std::max_element(max_list.begin(), max_list.end()));

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

    x_map.setPaintInterval(r.left(), r.right());
    y_map.setPaintInterval(r.bottom(), r.top());

    if (basic_line_color_changed_) {
        updateLineColors();
    }

    std::vector<QwtPlotCurve> curve(data_v_.size());
    for (std::size_t num_plot = 0; num_plot < data_v_.size(); ++num_plot) {
        painter.setRenderHint(QPainter::Antialiasing, curve[num_plot].testRenderHint(QwtPlotItem::RenderAntialiased));
        painter.setRenderHint(QPainter::HighQualityAntialiasing, curve[num_plot].testRenderHint(QwtPlotItem::RenderAntialiased));
        curve[num_plot].setBaseline(0.0);

        curve[num_plot].setPen(color_line_.at(num_plot), line_width_);
        curve[num_plot].setStyle(QwtPlotCurve::Lines);

        curve[num_plot].setBrush(QBrush(color_fill_, Qt::SolidPattern));

        curve[num_plot].setRawSamples(data_t_.data(), data_v_.at(num_plot).data(), data_t_.size());
        curve[num_plot].draw(&painter, x_map, y_map, r);
    }

    QwtLinearScaleEngine e;
    QwtPlotScaleItem scale_time(QwtScaleDraw::TopScale, y_map.s1());
    scale_time.setScaleDiv(e.divideScale(x_map.s1(), x_map.s2(), 10, 10));

    QwtPlotScaleItem scale_value(QwtScaleDraw::RightScale, x_map.s1());
    scale_value.setScaleDiv(e.divideScale(y_map.s1(), y_map.s2(), 10, 10));

    scale_time.draw(&painter, x_map, y_map, r);
    scale_value.draw(&painter, x_map, y_map, r);

    CvMatMessage::Ptr out_msg = std::make_shared<CvMatMessage>(enc::bgr, "plot", 0);
    out_msg->value = QtCvImageConverter::Converter::QImage2Mat(image);

    msg::publish(out_, out_msg);
}

double TimePlot::getLineWidth() const
{
    return line_width_;
}

const double* TimePlot::getTData() const
{
    return data_t_.data();
}
const double* TimePlot::getVData(std::size_t idx) const
{
    return data_v_.at(idx).data();
}
std::size_t TimePlot::getVDataCountNumCurves() const
{
    return data_v_.size();
}
std::size_t TimePlot::getCount() const
{
    return data_t_.size();
}
