#ifndef MEAN_DEV_NORMALIZATION_H
#define MEAN_DEV_NORMALIZATION_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class MeanStdDevNormalization : public Node
{
public:
    MeanStdDevNormalization();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:
    Input  *in_mean_;
    Input  *in_dev_;
    Input  *in_mat_;

    Output *out_;
};
}

#endif // MEAN_DEV_NORMALIZATION_H
