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
    csapex::ConnectorIn*    histogram_maxima_;
    csapex::ConnectorOut*   slices_;
    csapex::ConnectorIn*    matrix_;
};
}
#endif // SLICING_H
