#ifndef COL_H
#define COL_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class Col : public Node
{
public:
    Col();

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

#endif // COL_H
