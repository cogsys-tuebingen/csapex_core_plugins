#ifndef ROW_H
#define ROW_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class Row : public Node
{
public:
    Row();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:
    Input  *input_;
    Output *output_;

    bool request_center_;
    void requestCenter();
};
}

#endif // ROW_H
