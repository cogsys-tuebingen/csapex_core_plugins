#ifndef EXTRACT_TIMESTAMP_H_
#define EXTRACT_TIMESTAMP_H_

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class ExtractTimeStamp : public Node
{
public:
    ExtractTimeStamp();

    virtual void setup();
    virtual void process();

private:
    Input* input_;
    Output* output_;
};

}

#endif // EXTRACT_TIMESTAMP_H_
