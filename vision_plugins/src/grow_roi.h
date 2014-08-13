#ifndef GROW_ROI_H
#define GROW_ROI_H


/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class GrowROI : public csapex::Node
{
public:
    GrowROI();

    virtual void process();
    virtual void setup();

private:
    Input* input_;
    Output* output_;
};

}

#endif // GROW_ROI_H
