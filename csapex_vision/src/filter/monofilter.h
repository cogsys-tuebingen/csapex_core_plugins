#ifndef MONOFILTER_H
#define MONOFILTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {

class MonoFilter : public csapex::Node
{
public:
    MonoFilter();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* output_;
    csapex::Input*  input_;

    int   min_;
    int   max_;
    int   def_;
    bool  invert_;

    void  update();
};

}

#endif // MONOFILTER_H
