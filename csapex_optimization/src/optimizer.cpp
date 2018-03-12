/// HEADER
#include <csapex_optimization/optimizer.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/range_parameter.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/any_message.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/model/token.h>
#include <csapex/model/node_handle.h>
#include <csapex/profiling/interlude.hpp>

using namespace csapex;
using namespace csapex::connection_types;


Optimizer::Optimizer()
    : init_(false),
      optimization_running_(false),
      validation_running_(false),
      can_send_next_parameters_(false),
      last_fitness_(std::numeric_limits<double>::infinity()),
      best_fitness_(std::numeric_limits<double>::infinity()),
      worst_fitness_(-std::numeric_limits<double>::infinity())
{
}

void Optimizer::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("start"), [this](csapex::param::Parameter*) {
        start();
    });
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("finish"), [this](csapex::param::Parameter*) {
        finish();
    });

    parameters.addParameter(param::ParameterFactory::declareBool
                            ("finish_immediately",
                             param::ParameterDescription("Evaluate for every received fitness value, w/o relying on EndOfSequence."),
                             false),
                            evaluate_immediately_);

    parameters.addParameter(param::ParameterFactory::declareBool
                            ("perform_evaluation",
                             param::ParameterDescription("Set best parameters after optimization and re-run."),
                             false),
                            perform_evaluation_);

    stop_ = csapex::param::ParameterFactory::declareTrigger("stop").build<param::TriggerParameter>();
    parameters.addParameter(stop_, [this](csapex::param::Parameter*) {
        doStop();
    });

    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("set best"), [this](csapex::param::Parameter*) {
        stop();
        setBest();
    });
}

void Optimizer::setup(NodeModifier& node_modifier)
{
    slot_fitness_  = node_modifier.addSlot<GenericValueMessage<double>>("Fitness", [this](const TokenPtr& token) {
        if(!optimization_running_) {
            return;
        }

        auto m = token->getTokenData();

        if(std::dynamic_pointer_cast<connection_types::EndOfSequenceMessage const>(m)) {
            finish();

        } else {
            if(auto vm = std::dynamic_pointer_cast<connection_types::GenericValueMessage<double> const>(m)){
                fitness_ = vm->value;
            }

            if(evaluate_immediately_) {
                finish();

            } else {
                can_send_next_parameters_ = true;
                yield();
            }
        }
    });
    out_last_fitness_  = node_modifier.addOutput<double>("Last Fitness");
    out_best_fitness_  = node_modifier.addOutput<double>("Best Fitness");

    trigger_start_evaluation_ = node_modifier.addEvent("Evaluate");
    trigger_iteration_finished = node_modifier.addEvent<double>("Iteration finished");

    reset();
}

bool Optimizer::canProcess() const
{
    return validation_running_ || (optimization_running_ && can_send_next_parameters_);
}

void Optimizer::process()
{
    if(optimization_running_) {
        apex_assert(can_send_next_parameters_);
        can_send_next_parameters_ = false;
    }

    if(best_fitness_ != std::numeric_limits<double>::infinity()) {
        msg::publish(out_best_fitness_, best_fitness_);
    }
    if(last_fitness_ != std::numeric_limits<double>::infinity()) {
        msg::publish(out_last_fitness_, last_fitness_);
    }
}


void Optimizer::reset()
{
    Node::reset();

    last_fitness_ = std::numeric_limits<double>::infinity();
    best_fitness_ = std::numeric_limits<double>::infinity();
    worst_fitness_ = -std::numeric_limits<double>::infinity();
}

void Optimizer::start()
{
    node_handle_->setWarning("optimization started");
    reset();

    init_ = false;
    optimization_running_ = true;
    validation_running_ = false;
    can_send_next_parameters_ = true;


    msg::trigger(trigger_start_evaluation_);
    yield();
}


void Optimizer::stop()
{

    INTERLUDE("stop");
    stop_->trigger();

    connection_types::GenericValueMessage<double>::Ptr msg = std::make_shared<connection_types::GenericValueMessage<double>>();
    msg->value = best_fitness_;
    msg::trigger(trigger_iteration_finished, std::make_shared<Token>(msg));
}

void Optimizer::doStop()
{
    node_handle_->setWarning("optimization stopped, setting best values");

    optimization_running_ = false;

    setBest();

    if(perform_evaluation_) {
        validation_running_ = true;
        msg::trigger(trigger_start_evaluation_);
    }

    yield();
}

void Optimizer::finish()
{
    INTERLUDE("finish");
    if(validation_running_) {
        validation_running_ = false;
        return;
    }

    if(!optimization_running_) {
        return;
    }


    last_fitness_ = fitness_;

    if(worst_fitness_ < fitness_) {
        worst_fitness_ = fitness_;
    }

    if(best_fitness_ > fitness_) {

        INTERLUDE("update fitness");
        ainfo << "got better fitness: " << fitness_ << ", best was " << best_fitness_ << std::endl;
        best_fitness_ = fitness_;

        best_parameters_.clear();
        for(param::ParameterPtr p : getPersistentParameters()) {
            best_parameters_[p->getUUID()] = param::ParameterFactory::clone(p);
        }
    }

    INTERLUDE("generate and trigger");
    if(generateNextParameterSet()) {
        INTERLUDE("trigger");
        msg::trigger(trigger_start_evaluation_);
        can_send_next_parameters_ = true;
        yield();
    } else {
        stop();
    }
}

void Optimizer::setBest()
{
    for(param::ParameterPtr p : getPersistentParameters()) {
        p->setValueFrom(*best_parameters_.at(p->getUUID()));;
    }
}
