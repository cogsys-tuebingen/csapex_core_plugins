#ifndef CLOCK_H
#define CLOCK_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class Clock : public Node
{
private:
    enum Type
    {
        ZERO = 0,
        CURRENT = 1,
        CHRONO = 2
    };

public:
    Clock();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Output* output_;
};

}  // namespace csapex

#endif  // CLOCK_H
