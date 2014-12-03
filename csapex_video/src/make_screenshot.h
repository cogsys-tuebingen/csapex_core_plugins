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

    void setupParameters();
    void setup();
    void process();

private:
    void makeScreenshot();

private:
    Slot* in_;
};


}

#endif // MAKE_SCREENSHOT_H
