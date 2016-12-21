/// HEADER
#include "scatter_plot.h"

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

CSAPEX_REGISTER_CLASS(csapex::ScatterPlot, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

void ScatterPlot::setup(NodeModifier &node_modifier)
{
    in_x_  = node_modifier.addMultiInput<double, GenericVectorMessage>("x");
    in_y_  = node_modifier.addMultiInput<double, GenericVectorMessage>("y");
}


void ScatterPlot::setupParameters(Parameterizable &parameters)
{
    Plot::setupParameters(parameters);

    parameters.addParameter(param::ParameterFactory::declareRange("point_size", 0.1, 10.0, 1.0, 0.01));
    parameters.addParameter(param::ParameterFactory::declareTrigger("reset"),
                            [this](param::Parameter*) {
        reset();
    });
    reset();
}

void ScatterPlot::reset()
{
    x.clear();
    y.clear();

    max_x_ = max_y_ = -std::numeric_limits<double>::infinity();
    min_x_ = min_y_ = std::numeric_limits<double>::infinity();
}

void ScatterPlot::process()
{
    apex_assert(msg::isValue<double>(in_x_) == msg::isValue<double>(in_y_));

    if(msg::isValue<double>(in_x_)) {
        apex_assert(msg::isValue<double>(in_y_));
        double value_x = msg::getValue<double>(in_x_);
        double value_y = msg::getValue<double>(in_y_);

        x.push_back(value_x);
        y.push_back(value_y);

        min_x_ = std::min(min_x_, value_x);
        max_x_ = std::max(max_x_, value_x);
        min_y_ = std::min(min_y_, value_y);
        max_y_ = std::max(max_y_, value_y);
    }
    else {
        GenericVectorMessage::ConstPtr message_x = msg::getMessage<GenericVectorMessage>(in_x_);
        apex_assert(std::dynamic_pointer_cast<GenericValueMessage<double>>(message_x->nestedType()));
        GenericVectorMessage::ConstPtr message_y = msg::getMessage<GenericVectorMessage>(in_y_);
        apex_assert(std::dynamic_pointer_cast<GenericValueMessage<double>>(message_y->nestedType()));

        apex_assert(message_x->nestedValueCount() == message_y->nestedValueCount());

        for(std::size_t i = 0, n = message_x->nestedValueCount(); i < n; ++i){
            if(auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(message_x->nestedValue(i))) {
                x.push_back(pval->value);
                min_x_ = std::min(min_x_, pval->value);
                max_x_ = std::max(max_x_, pval->value);
            }
            if(auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(message_y->nestedValue(i))) {
                y.push_back(pval->value);
                min_y_ = std::min(min_y_, pval->value);
                max_y_ = std::max(max_y_, pval->value);
            }
        }

    }

    x_map.setScaleInterval(min_x_ - 1, max_x_ + 1);
    y_map.setScaleInterval(min_y_ - 1, max_y_ + 1);

    display_request();
}

const double* ScatterPlot::getXData() const
{
    return x.data();
}
const double* ScatterPlot::getYData() const
{
    return y.data();
}
std::size_t ScatterPlot::getCount() const
{
    return x.size();
}
