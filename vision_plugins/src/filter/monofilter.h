#ifndef MONOFILTER_H
#define MONOFILTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {

class MonoFilter : public csapex::Node
{
public:
    MonoFilter();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    csapex::ConnectorOut* output_;
    csapex::ConnectorIn*  input_;

    int   min_;
    int   max_;
    int   def_;
    bool  invert_;

    void  update();
};

}

#endif // MONOFILTER_H
