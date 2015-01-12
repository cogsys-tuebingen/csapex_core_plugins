#ifndef TRANSFORM_TO_ODOMETRY_H
#define TRANSFORM_TO_ODOMETRY_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class TransformToOdometry : public csapex::Node
{
public:
    TransformToOdometry();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_;
    Output* out_;
};


}

#endif // TRANSFORM_TO_ODOMETRY_H
