#ifndef SLICING_H
#define SLICING_H


/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class Slicing : public csapex::Node
{
public:
    Slicing();

    virtual void process();
    virtual void setup();

protected:
    csapex::Input*    histogram_maxima_;
    csapex::Output*   slices_;
    csapex::Input*    matrix_;
};
}
#endif // SLICING_H
