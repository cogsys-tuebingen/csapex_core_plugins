/// HEADER
#include "scatter_plot.h"

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
#include <libqwt/qwt_scale_engine.h>

CSAPEX_REGISTER_CLASS(csapex::ScatterPlot, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

void ScatterPlot::setup(NodeModifier& node_modifier)
{
    in_x_ = node_modifier.addMultiInput<double, GenericVectorMessage>("x");
    in_y_ = node_modifier.addMultiInput<double, GenericVectorMessage>("y");
    setupVariadic(node_modifier);
}

void ScatterPlot::setupParameters(Parameterizable& parameters)
{
    Plot::setupParameters(parameters);

    setupVariadicParameters(parameters);

    parameters.addParameter(param::factory::declareRange("point_size", 0.1, 10.0, 1.0, 0.01));
    parameters.addParameter(param::factory::declareTrigger("reset"), [this](param::Parameter*) { reset(); });
    reset();
}

void ScatterPlot::reset()
{
    std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
    x.clear();
    y.clear();
    for (std::vector<double>& var : var_y_) {
        var.clear();
    }

    max_x_ = max_y_ = -std::numeric_limits<double>::infinity();
    min_x_ = min_y_ = std::numeric_limits<double>::infinity();
}

void ScatterPlot::process()
{
    apex_assert(msg::isValue<double>(in_x_) == msg::isValue<double>(in_y_));
    std::size_t num_inputs = 0;
    {
        std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
        num_inputs = VariadicInputs::getVariadicInputCount();
        num_plots_ = num_inputs + 1;

        for (std::size_t i_inputs = 0; i_inputs < num_inputs; ++i_inputs) {
            InputPtr in = VariadicInputs::getVariadicInput(i_inputs);
            if (!msg::isConnected(in.get())) {
                --num_plots_;
            } else {
                //            apex_assert(msg::isValue<double>(in.get()) ==
                //            msg::isValue<double>(in_y_));
            }
        }

        updateLineColors();
        var_y_.resize(num_plots_ - 1);
    }
    if (msg::isValue<double>(in_x_)) {
        apex_assert(msg::isValue<double>(in_y_));
        std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
        double value_x = msg::getValue<double>(in_x_);
        double value_y = msg::getValue<double>(in_y_);

        x.push_back(value_x);
        y.push_back(value_y);

        min_x_ = std::min(min_x_, value_x);
        max_x_ = std::max(max_x_, value_x);
        min_y_ = std::min(min_y_, value_y);
        max_y_ = std::max(max_y_, value_y);

        for (std::size_t i_inputs = 0; i_inputs < num_inputs; ++i_inputs) {
            InputPtr in = VariadicInputs::getVariadicInput(i_inputs);
            if (msg::isConnected(in.get())) {
                double y_i = msg::getValue<double>(in.get());

                var_y_.at(i_inputs).emplace_back(y_i);

                max_y_ = std::max(max_y_, y_i);
                min_y_ = std::min(min_y_, y_i);
            }
        }

    } else {
        std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
        GenericVectorMessage::ConstPtr message_x = msg::getMessage<GenericVectorMessage>(in_x_);
        apex_assert(std::dynamic_pointer_cast<GenericValueMessage<double>>(message_x->nestedType()));
        GenericVectorMessage::ConstPtr message_y = msg::getMessage<GenericVectorMessage>(in_y_);
        apex_assert(std::dynamic_pointer_cast<GenericValueMessage<double>>(message_y->nestedType()));

        apex_assert(message_x->nestedValueCount() == message_y->nestedValueCount());

        for (std::size_t i = 0, n = message_x->nestedValueCount(); i < n; ++i) {
            if (auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(message_x->nestedValue(i))) {
                x.push_back(pval->value);
                min_x_ = std::min(min_x_, pval->value);
                max_x_ = std::max(max_x_, pval->value);
            }
            if (auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(message_y->nestedValue(i))) {
                y.push_back(pval->value);
                min_y_ = std::min(min_y_, pval->value);
                max_y_ = std::max(max_y_, pval->value);
            }
        }

        std::size_t data_counter = 0;
        for (std::size_t i_inputs = 0; i_inputs < num_inputs; ++i_inputs) {
            InputPtr in = VariadicInputs::getVariadicInput(i_inputs);
            if (msg::isConnected(in.get())) {
                GenericVectorMessage::ConstPtr message_i = msg::getMessage<GenericVectorMessage>(in.get());

                apex_assert(std::dynamic_pointer_cast<GenericValueMessage<double>>(message_i->nestedType()));
                apex_assert(message_x->nestedValueCount() == message_i->nestedValueCount());
                //                var_y_[data_counter].resize(message_i->nestedValueCount());

                for (std::size_t n_point = 0; n_point < message_i->nestedValueCount(); ++n_point) {
                    auto pval = std::dynamic_pointer_cast<GenericValueMessage<double> const>(message_i->nestedValue(n_point));
                    var_y_[data_counter].push_back(pval->value);
                    min_y_ = std::min(min_y_, pval->value);
                    max_y_ = std::max(max_y_, pval->value);
                }
                ++data_counter;
            }
        }
    }

    x_map.setScaleInterval(min_x_ - 1, max_x_ + 1);
    y_map.setScaleInterval(min_y_ - 1, max_y_ + 1);

    display_request();
}

const double* ScatterPlot::getXData() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
    return x.data();
}
const double* ScatterPlot::getYData() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
    return y.data();
}
std::size_t ScatterPlot::getCount() const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
    return x.size();
}

const double* ScatterPlot::getVarData(std::size_t idx) const
{
    std::unique_lock<std::recursive_mutex> lock(mutex_buffer_);
    return var_y_.at(idx).data();
}
