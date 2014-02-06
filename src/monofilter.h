#ifndef MONOFILTER_H
#define MONOFILTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

class MonoFilter : public csapex::Node
{
public:
    MonoFilter();

    virtual void allConnectorsArrived();
    virtual void setup();

protected:
    ConnectorOut*                    output_;
    ConnectorIn*                     input_;

    void update();
    int   min_;
    int   max_;
    int   def_;
    bool  invert_;
};

}

#endif // MONOFILTER_H
