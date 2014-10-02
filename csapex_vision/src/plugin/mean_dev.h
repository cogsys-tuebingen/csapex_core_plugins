#ifndef MEAN_H
#define MEAN_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class MeanStdDev : public Node
{
public:
    MeanStdDev();

    virtual void setup();
    virtual void setupParameters();
    virtual void process();

private:

    Input  *in_mat_;
    Input  *in_mask_;
    Output *out_mean_;
    Output *out_stddev_;
};
}
#endif // MEAN_H
