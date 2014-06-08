#ifndef TIME_OFFSET_H_
#define TIME_OFFSET_H_

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TimeOffset : public csapex::Node
{
public:
    TimeOffset();

    virtual void process();
    virtual void setup();

private:
    ConnectorOut* output_;
    ConnectorIn* input_;
};

}

#endif // TIME_OFFSET_H_
