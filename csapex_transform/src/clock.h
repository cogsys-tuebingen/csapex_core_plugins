#ifndef CLOCK_H
#define CLOCK_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/ros_handler.h>

namespace csapex {

class Clock : public Node
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
