/// HEADER
#include "plot.h"

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

Plot::Plot()
    : width_(640),
      height_(320)
{

}

Plot::~Plot()
{

}

void Plot::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareColorParameter(
                                "~plot/color/background", 255, 255, 255),
                            setColor(color_bg_));
    parameters.addParameter(param::ParameterFactory::declareColorParameter(
                                "~plot/color/fill", 200, 200, 200),
                            setColor(color_fill_));
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


std::function<void(param::Parameter*)> Plot::setColor(QColor& color) {
    return [&](param::Parameter* p) {
        std::vector<int> c = p->as<std::vector<int>>();
        color.setRed(c[0]);
        color.setGreen(c[1]);
        color.setBlue(c[2]);

        update();
    };
}

