#ifndef ABSOLUTE_DIFFERENCE_H
#define ABSOLUTE_DIFFERENCE_H

/// PROJECT
#include <csapex/model/node.h>

namespace vision_plugins {
class AbsoluteDifference : public csapex::Node
{
public:
    AbsoluteDifference();

    virtual void process();
    virtual void setup();

protected:
    csapex::ConnectorIn     *mat_1_in_;
    csapex::ConnectorIn     *mat_2_in_;
    csapex::ConnectorOut    *result_;
};
}
#endif // ABSOLUTE_DIFFERENCE_H
