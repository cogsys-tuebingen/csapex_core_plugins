#ifndef BF_OPTIMIZER_H
#define BF_OPTIMIZER_H

/// COMPONENT
#include <csapex/msg/msg_fwd.h>

/// PROJECT
#include <csapex/model/tickable_node.h>

/// SYSTEM

namespace csapex {


class BFOptimizer : public csapex::TickableNode
{
    friend class BFOptimizerAdapter;

public:
    BFOptimizer();

    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    virtual bool canTick() override;
    virtual bool tick(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters) override;

    virtual void processMarker(const csapex::connection_types::MessageConstPtr &marker) override;

    int stepsNecessary();
    void restart();

public:
    csapex::slim_signal::Signal<void(int)> step;

private:
    void start();
    void stop();
    void doStop();

    void finish();

    bool nextStep();
    bool increaseParameter(std::size_t i);

private:
    Input* in_;
    Output* out_last_fitness_;
    Output* out_best_fitness_;
    Event* trigger_start_evaluation_;
    Event* trigger_iteration_finished;

    param::TriggerParameterPtr stop_;

    double fitness_;
    double last_fitness_;
    double best_fitness_;

    bool init_;
    bool running_;
    bool next_tick_;

    int sent_;

    int step_;
    std::vector<int> current_index_;
};


}

#endif // BF_OPTIMIZER_H
