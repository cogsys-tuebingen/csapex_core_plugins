#ifndef OPTIMIZER_H
#define OPTIMIZER_H

/// COMPONENT
#include <csapex/msg/msg_fwd.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <unordered_map>

namespace csapex {


class Optimizer : public csapex::Node
{
public:
    Optimizer();

    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    virtual bool canProcess() const override;

    virtual void reset() override;

    void setBest();

    virtual void start();
    virtual void stop();

protected:
    virtual void doStop();

    virtual void finish();

    virtual bool generateNextParameterSet() = 0;

private:
    Slot* slot_fitness_;
    Output* out_last_fitness_;
    Output* out_best_fitness_;
    Event* trigger_start_evaluation_;
    Event* trigger_iteration_finished;

    param::TriggerParameterPtr stop_;

    double last_fitness_;
    double best_fitness_;

    bool init_;
    bool optimization_running_;
    bool validation_running_;
    bool can_send_next_parameters_;

    std::unordered_map<UUID, param::Parameter::Ptr, UUID::Hasher> best_parameters_;

protected:
    double fitness_;
};


}

#endif // OPTIMIZER_H
