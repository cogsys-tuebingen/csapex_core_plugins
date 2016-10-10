/// HEADER
#include "vector_plot.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
#include <qwt_scale_engine.h>


using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(csapex::VectorPlot, csapex::Node)

VectorPlot::VectorPlot():
    initialize_(true),
    basic_line_color_changed_(true),
    has_time_in_(false),
    num_plots_(1),
    width_(640),
    height_(320)
{

}

void VectorPlot::setup(NodeModifier &node_modifier)
{

    setupVariadic(node_modifier);
    in_time_  = node_modifier.addOptionalInput<GenericVectorMessage, double>("time");

    out_ = node_modifier.addOutput<CvMatMessage>("Plot");

    color_line_.resize(num_plots_);
}

void VectorPlot::setupParameters(Parameterizable &parameters)
{

    setupVariadicParameters(parameters);

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

    std::function<void(param::Parameter* p)> setLineColors= [this](param::Parameter* p){
        std::vector<int> c = p->as<std::vector<int>>();
        basic_line_color_.setRed(c[0]);
        basic_line_color_.setGreen(c[1]);
        basic_line_color_.setBlue(c[2]);
        calculateLineColors();
        basic_line_color_changed_ = true;
        update();
    };


    parameters.addParameter(param::ParameterFactory::declareColorParameter(
                                "~plot/color/background", 255, 255, 255),
                            setColor(color_bg_));
    parameters.addParameter(param::ParameterFactory::declareColorParameter(
                                "~plot/color/line", 100, 100, 255),
                            setLineColors);
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


    color_fill_.setAlpha(100);
}

void VectorPlot::process()
{
    std::size_t num_inputs = VariadicInputs::getVariadicInputCount();
    num_plots_ = num_inputs;;

    for(std::size_t i_inputs = 0; i_inputs < num_plots_; ++ i_inputs){
        InputPtr in = VariadicInputs::getVariadicInput(i_inputs);
        if(!msg::isConnected(in.get())){
            --num_plots_;
        }

    }

    data_v_.resize(num_plots_);
    calculateLineColors();

    std::size_t data_counter = 0;

    for(std::size_t i_inputs = 0; i_inputs < num_inputs; ++i_inputs){

        InputPtr in = VariadicInputs::getVariadicInput(i_inputs);
        if(msg::isConnected(in.get())){
            GenericVectorMessage::ConstPtr message = msg::getMessage<GenericVectorMessage>(in.get());

            apex_assert(std::dynamic_pointer_cast<GenericValueMessage<double>>(message->nestedType()));

            data_v_[data_counter].resize(message->nestedValueCount());

            for(std::size_t n_point = 0; n_point <message->nestedValueCount(); ++n_point){
                auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(message->nestedValue(n_point));
                data_v_[data_counter][n_point] = pval->value;
            }
//            if(data_counter > 0){
//                apex_assert(data_v_[data_counter].size() == data_v_[data_counter -1].size());
//            }
            ++data_counter;
        }
    }

    data_t_raw_.clear();
    if(msg::hasMessage(in_time_)){
        has_time_in_ = true;
        std::shared_ptr<std::vector<double> const> time = msg::getMessage<GenericVectorMessage, double>(in_time_);
        data_t_raw_.resize(time->size());
        for(std::size_t n = 0; n < time->size(); ++n){
            data_t_raw_[n] = time->at(n);
        }

        apex_assert(data_t_raw_.size() == data_v_.front().size());
    }
    else{
        has_time_in_ = false;
        data_t_.resize(data_v_.front().size());
        for(std::size_t i = 0; i < data_t_.size();++i){
            data_t_[i] = i;
        }
    }

    preparePlot();

    if(msg::isConnected(out_)) {
        renderAndSend();
    }

    display_request();
}

void VectorPlot::reset()
{
    data_v_.clear();
    data_t_.clear();
    data_t_raw_.clear();
    initialize_ = true;
    num_plots_ = 1;
    init();
}

void VectorPlot::init()
{
    has_time_in_ = false;
    auto now = std::chrono::system_clock::now();
    start_t_ = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

void VectorPlot::preparePlot()
{
    color_fill_.setAlpha(100);

    if(has_time_in_){
        std::size_t n = data_t_raw_.size();
        data_t_.resize(n, 0.0);

        double time_offset = time_relative_ ? data_t_raw_.front() : 0.0;
        double time_scale = time_seconds_ ? 1e-6 : 1.0;

        for(std::size_t i = 0; i < n; ++i) {
            data_t_[i] = (data_t_raw_[i] - time_offset) * time_scale;
        }
    }

    std::vector<double> min_list;
    std::vector<double> max_list;
    for(auto data: data_v_) {
        if(data.size() != 0) {
            min_list.push_back(std::min(0.0, *std::min_element(data.begin(), data.end())));
            max_list.push_back(std::max(0.0, *std::max_element(data.begin(), data.end())));
        }
    }

    double min = std::min(0.0, *std::min_element(min_list.begin(), min_list.end()));
    double max = std::max(0.0, *std::max_element(max_list.begin(), max_list.end()));

    x_map.setScaleInterval(data_t_.front(), data_t_.back());


    y_map.setScaleInterval(min - 1, max + 1);
}

void VectorPlot::renderAndSend()
{
    QImage image(width_, height_, QImage::Format_RGB32);
    QPainter painter;
    painter.begin(&image);
    painter.fillRect(image.rect(), color_bg_);

    QRect r(0, 0, image.width(), image.height());

    x_map.setPaintInterval( r.left(), r.right() );
    y_map.setPaintInterval( r.bottom(), r.top() );

    if(basic_line_color_changed_) {
        calculateLineColors();
    }

    std::vector<double> x_data;

    QwtPlotCurve curve[data_v_.size()];
    for(std::size_t  num_plot= 0; num_plot < data_v_.size(); ++num_plot) {

        painter.setRenderHint( QPainter::Antialiasing,
                               curve[num_plot].testRenderHint( QwtPlotItem::RenderAntialiased ) );
        painter.setRenderHint( QPainter::HighQualityAntialiasing,
                               curve[num_plot].testRenderHint( QwtPlotItem::RenderAntialiased ) );
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


    CvMatMessage::Ptr out_msg = std::make_shared<CvMatMessage>(enc::bgr, 0);
    out_msg->value = QtCvImageConverter::Converter<QImage>::QImage2Mat(image);

    msg::publish(out_, out_msg);
}

int VectorPlot::getWidth() const
{
    return width_;
}

int VectorPlot::getHeight() const
{
    return height_;
}

double VectorPlot::getLineWidth() const
{
    return line_width_;
}

QColor VectorPlot::getBackgroundColor() const
{
    return color_bg_;
}
QColor VectorPlot::getLineColor(std::size_t idx) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return color_line_[idx];
}
QColor VectorPlot::getFillColor() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return color_fill_;
}

const double* VectorPlot::getTData() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return data_t_.data();
}
const double* VectorPlot::getVData(std::size_t idx) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return data_v_.at(idx).data();
}
std::size_t VectorPlot::getVDataCountNumCurves() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    return data_v_.size();
}
std::size_t VectorPlot::getCount() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    std::vector<std::size_t> min_sz(1+data_v_.size());
    min_sz[0] = data_t_.size();
    for(std::size_t i = 0; i < data_v_.size(); ++i){
        min_sz[i+1] = data_v_[i].size();
    }
    std::size_t res  = std::min(data_t_.size(), *std::min_element(min_sz.begin(), min_sz.end()));
    return res;
}

const QwtScaleMap& VectorPlot::getXMap() const
{
    return x_map;
}
const QwtScaleMap& VectorPlot::getYMap() const
{
    return y_map;
}

Input* VectorPlot::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool /*optional*/)
{
    return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::GenericVectorMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
}

void VectorPlot::calculateLineColors()
{
    int r,g,b,a;
    basic_line_color_.getRgb(&r,&g,&b,&a);
    QColor color;
    color.setRed(r);
    color.setGreen(g);
    color.setBlue(b);
    int h,s,v;
    color.getHsv(&h,&s,&v);

    for(std::size_t i = 0; i < num_plots_; ++i) {

        double hnew =  h + i * 360/num_plots_;
        while (hnew > 359 || hnew < 0) {
            if(hnew > 359) {
                hnew -= 360;
            }
            else if(hnew < 0) {
                hnew += 360;
            }

        }
        color.setHsv(hnew,s,v);
        color_line_[i] = color;
    }

    basic_line_color_changed_ = false;
    update();
}
