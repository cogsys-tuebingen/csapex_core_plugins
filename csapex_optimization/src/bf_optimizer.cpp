/// HEADER
#include "bf_optimizer.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/model/token.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::BFOptimizer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

BFOptimizer::BFOptimizer()
{
}

int BFOptimizer::stepsNecessary()
{
    int steps = 1;
    for (csapex::param::Parameter::Ptr p : getPersistentParameters()) {
        param::RangeParameter::Ptr dbl_range = std::dynamic_pointer_cast<param::RangeParameter>(p);
        if (!dbl_range || !dbl_range->is<double>()) {
            continue;
        }

        steps *= std::ceil((dbl_range->max<double>() - dbl_range->min<double>()) / dbl_range->step<double>() + 1);
    }
    return steps;
}

void BFOptimizer::reset()
{
    Optimizer::reset();

    step_ = 0;
    current_index_.resize(getPersistentParameters().size(), 0);

    for (auto p : getPersistentParameters()) {
        param::RangeParameter::Ptr range_p = std::dynamic_pointer_cast<param::RangeParameter>(p);
        if (range_p) {
            if (range_p->is<double>()) {
                range_p->set(range_p->min<double>());
            } else if (range_p->is<int>()) {
                range_p->set(range_p->min<int>());
            }
        }
    }
}

bool BFOptimizer::generateNextParameterSet()
{
    step(step_);

    return increaseParameter(0);
}

bool BFOptimizer::increaseParameter(std::size_t i)
{
    std::vector<csapex::param::Parameter::Ptr> params = getPersistentParameters();
    if (i >= params.size() || i >= current_index_.size()) {
        return false;
    }

    param::RangeParameter::Ptr dbl_range = std::dynamic_pointer_cast<param::RangeParameter>(params[i]);
    if (!dbl_range || !dbl_range->is<double>()) {
        return false;
    }

    double v = dbl_range->as<double>();
    if (v < dbl_range->max<double>()) {
        ++current_index_[i];
        dbl_range->set<double>(dbl_range->min<double>() + current_index_[i] * dbl_range->step<double>());
        ++step_;
        return true;
    } else {
        dbl_range->set<double>(dbl_range->min<double>());
        current_index_[i] = 0;
        return increaseParameter(i + 1);
    }
}

void BFOptimizer::doStop()
{
    step(stepsNecessary());
    Optimizer::doStop();
}
