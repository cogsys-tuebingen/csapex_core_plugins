#ifndef GRAB_CUT_H
#define GRAB_CUT_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class GrabCut : public csapex::Node
{
public:
    GrabCut();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_;
    Input* in_fg_;
    Input* in_bg_;

    Input* in_roi_;

    Output* out_fg_;
    Output* out_bg_;
};


}

#endif // GRAB_CUT_H
