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

    void process();
    void setup();

    virtual void tick();

private:
    ConnectorOut* output_;
};

}

#endif // CLOCK_H
