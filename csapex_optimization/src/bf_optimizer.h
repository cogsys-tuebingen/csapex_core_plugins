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
    virtual void tick() override;

    int stepsNecessary();

public:
    boost::signals2::signal<void(int)> step;

private:
    void start();
    void stop();

    bool nextStep();
    bool increaseParameter(std::size_t i);

private:
    Input* in_;
    Output* out_;

    bool init_;
    bool running_;
    int step_;
    std::vector<int> current_index_;
};


}

#endif // BF_OPTIMIZER_H
