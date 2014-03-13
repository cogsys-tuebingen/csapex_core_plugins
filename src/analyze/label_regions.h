#ifndef LABEL_REGIONS_H
#define LABEL_REGIONS_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {

class LabelRegions : public csapex::Node
{
public:
    LabelRegions();

    virtual void process();
    virtual void setup();

protected:
    csapex::ConnectorOut*   output_;
    csapex::ConnectorIn*    input_;
};
}

#endif // LABEL_REGIONS_H
