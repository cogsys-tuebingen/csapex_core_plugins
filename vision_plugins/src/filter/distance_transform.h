#ifndef DISTANCE_TRANSFORM_H
#define DISTANCE_TRANSFORM_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class DistanceTransform : public csapex::Node
{
public:
    DistanceTransform();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_;
    Output* out_;
    Output* out_label_;
};


}

#endif // DISTANCE_TRANSFORM_H
