/// HEADER
#include "bf_optimizer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <utils_param/range_parameter.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::BFOptimizer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


BFOptimizer::BFOptimizer()
    : init_(false), running_(false)
{
}

void BFOptimizer::setupParameters()
{
}

void BFOptimizer::setup()
{
    in_  = modifier_->addInput<double>("Fitness");
    out_ = modifier_->addOutput<AnyMessage>("Trigger");


    //    modifier_->setIsSource(true);
    modifier_->setIsSink(true);
}

void BFOptimizer::tick()
{
    // initilization?
    if(!init_) {
        current_index_.clear();
        foreach(param::Parameter::Ptr p, getParameters()) {
            param::RangeParameter::Ptr dbl_range = boost::dynamic_pointer_cast<param::RangeParameter>(p);
            if(!dbl_range || !dbl_range->is<double>()) {
                continue;
            }
            current_index_.push_back(0);
            dbl_range->set<double>(dbl_range->min<double>());
        }

        step_ = 0;
        init_ = true;
    }

    if(nextStep()) {
        apex_assert_hard(!running_);
        running_ = true;

        AnyMessage::Ptr trigger(new AnyMessage);
        out_->publish(trigger);
    } else {
        stop();
    }
}

int BFOptimizer::stepsNecessary()
{
    int steps = 1;
    foreach(param::Parameter::Ptr p, getParameters()) {
        param::RangeParameter::Ptr dbl_range = boost::dynamic_pointer_cast<param::RangeParameter>(p);
        if(!dbl_range || !dbl_range->is<double>()) {
            continue;
        }

        steps *= std::ceil((dbl_range->max<double>() - dbl_range->min<double>()) / dbl_range->step<double>() + 1);
    }
    return steps;
}

bool BFOptimizer::nextStep()
{
    step(step_);

    return increaseParameter(0);
}

bool BFOptimizer::increaseParameter(std::size_t i)
{
    std::vector<param::Parameter::Ptr> params = getParameters();
    if(i > params.size()) {
        return false;
    }

    param::RangeParameter::Ptr dbl_range = boost::dynamic_pointer_cast<param::RangeParameter>(params[i]);
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
    apex_assert_hard(running_);
    running_ = false;

    double fitness = in_->getValue<double>();
    ainfo << "got fitness: " << fitness << std::endl;
}

void BFOptimizer::start()
{
    ainfo << "starting optimization" << std::endl;
    init_ = false;
    modifier_->setTickEnabled(true);
    modifier_->setTickFrequency(100.);
}


void BFOptimizer::stop()
{
    step(stepsNecessary());
    ainfo << "stopping optimization" << std::endl;
    modifier_->setTickEnabled(false);
}
