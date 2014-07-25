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
    ConnectorIn* in_;
    ConnectorIn* in_fg_;
    ConnectorIn* in_bg_;

    ConnectorIn* in_roi_;

    ConnectorOut* out_fg_;
    ConnectorOut* out_bg_;
};


}

#endif // GRAB_CUT_H
