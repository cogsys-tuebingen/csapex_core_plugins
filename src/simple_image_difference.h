#ifndef SIMPLE_IMAGE_DIFFERENCE_H
#define SIMPLE_IMAGE_DIFFERENCE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins
{

class SimpleImageDifference : public csapex::Node
{
public:
    SimpleImageDifference();

public:
    void allConnectorsArrived();
    void setup();

private:
    csapex::ConnectorIn* in_a_;
    csapex::ConnectorIn* in_b_;
    csapex::ConnectorOut* out_;
};

}

#endif // SIMPLE_IMAGE_DIFFERENCE_H
