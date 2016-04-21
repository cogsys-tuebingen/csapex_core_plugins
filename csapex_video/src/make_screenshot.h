#ifndef MAKE_SCREENSHOT_H
#define MAKE_SCREENSHOT_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class MakeScreenshot : public csapex::Node
{
public:
    MakeScreenshot();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    void makeScreenshot();

private:
    Slot* in_;
    Event* done_;
};


}

#endif // MAKE_SCREENSHOT_H
