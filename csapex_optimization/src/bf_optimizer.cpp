/// HEADER
#include "bf_optimizer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/range_parameter.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/any_message.h>
#include <csapex/param/trigger_parameter.h>

CSAPEX_REGISTER_CLASS(csapex::BFOptimizer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


BFOptimizer::BFOptimizer()
    : last_fitness_(std::numeric_limits<double>::infinity()),
      best_fitness_(std::numeric_limits<double>::infinity()),
      init_(false),
      running_(false)
{
}

void BFOptimizer::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("start"), [this](csapex::param::Parameter*) {
        start();
    });
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("finish"), [this](csapex::param::Parameter*) {
        finish();
    });
    stop_ = csapex::param::ParameterFactory::declareTrigger("stop").build<param::TriggerParameter>();
    parameters.addParameter(stop_, [this](csapex::param::Parameter*) {
        doStop();
    });
}

void BFOptimizer::setup(NodeModifier& node_modifier)
{
    in_  = node_modifier.addInput<double>("Fitness");
    out_last_fitness_  = node_modifier.addOutput<double>("Last Fitness");
    out_best_fitness_  = node_modifier.addOutput<double>("Best Fitness");

    trigger_start_evaluation_ = node_modifier.addTrigger("Evaluate");

    node_modifier_->setIsSource(true);
    node_modifier_->setIsSink(true);

    restart();
}

void BFOptimizer::tick()
{
    if(!running_) {
        return;
    }

//    if(!nextStep()) {
//        stop();
//    }
}

int BFOptimizer::stepsNecessary()
{
    int steps = 1;
    for(csapex::param::Parameter::Ptr p : getPersistentParameters()) {
        param::RangeParameter::Ptr dbl_range = std::dynamic_pointer_cast<param::RangeParameter>(p);
        if(!dbl_range || !dbl_range->is<double>()) {
            continue;
        }

        steps *= std::ceil((dbl_range->max<double>() - dbl_range->min<double>()) / dbl_range->step<double>() + 1);
    }
    return steps;
}

void BFOptimizer::restart()
{
    step_ = 0;
    current_index_.resize(getPersistentParameters().size(), 0);
}

bool BFOptimizer::nextStep()
{
    step(step_);

    return increaseParameter(0);
}

bool BFOptimizer::increaseParameter(std::size_t i)
{
    std::vector<csapex::param::Parameter::Ptr> params = getPersistentParameters();
    if(i >= params.size() || i >= current_index_.size()) {
        return false;
    }

    param::RangeParameter::Ptr dbl_range = std::dynamic_pointer_cast<param::RangeParameter>(params[i]);
    if(!dbl_range || !dbl_range->is<double>()) {
        return false;
    }

    double v = dbl_range->as<double>();
    if(v < dbl_range->max<double>()) {
        ++current_index_[i];
        dbl_range->set<double>(dbl_range->min<double>() + current_index_[i] * dbl_range->step<double>());
        ++step_;
        return true;
    } else {
        dbl_range->set<double>(dbl_range->min<double>());
        current_index_[i] = 0;
        return increaseParameter(i+1);
    }
}

void BFOptimizer::process()
{
    if(!running_) {
        return;
    }

    fitness_ = msg::getValue<double>(in_);

    if(best_fitness_ != std::numeric_limits<double>::infinity()) {
        msg::publish(out_best_fitness_, best_fitness_);
    }
    if(last_fitness_ != std::numeric_limits<double>::infinity()) {
        msg::publish(out_last_fitness_, last_fitness_);
    }

    ainfo << "got fitness: " << fitness_ << std::endl;

}

void BFOptimizer::start()
{
    ainfo << "starting optimization" << std::endl;
    init_ = false;
    running_ = true;

    setTickEnabled(true);
    setTickFrequency(100.);
}


void BFOptimizer::stop()
{
    stop_->trigger();
}

void BFOptimizer::doStop()
{
    step(stepsNecessary());
    ainfo << "stopping optimization" << std::endl;
    setTickEnabled(false);
    running_ = false;
}

void BFOptimizer::finish()
{
    if(!running_) {
        return;
    }

    ainfo << "got fitness: " << fitness_ << std::endl;

    last_fitness_ = fitness_;

    if(fitness_ < best_fitness_) {
        best_fitness_ = fitness_;
    }

    if(!nextStep()) {
        stop();
    }
}
