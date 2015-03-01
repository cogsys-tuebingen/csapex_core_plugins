#ifndef CLOCK_H
#define CLOCK_H

/// PROJECT
#include <csapex_ros/ros_node.h>

namespace csapex {

class Clock : public RosNode
{
private:
    enum Type {
        ZERO = 0,
        CURRENT = 1
    };

public:    
    Clock();

    virtual void setupROS() override;
    virtual void processROS() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

    virtual void tick() override;

private:
    Output* output_;
};

}

#endif // CLOCK_H
