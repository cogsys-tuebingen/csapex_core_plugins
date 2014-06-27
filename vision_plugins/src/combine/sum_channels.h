#ifndef SUM_CHANNELS_H
#define SUM_CHANNELS_H

/// COMPONENT
#include <csapex/model/node.h>
namespace vision_plugins {
class SumChannels : public csapex::Node
{
public:
    SumChannels();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    csapex::ConnectorOut*            output_;
    csapex::ConnectorIn*             input_;

};
}
#endif // SUM_CHANNELS_H
