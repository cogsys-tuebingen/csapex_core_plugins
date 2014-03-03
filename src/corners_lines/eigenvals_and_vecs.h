#ifndef EIGENVALS_AND_VECS_H
#define EIGENVALS_AND_VECS_H

/// COMPONENT
#include "detection.h"

namespace vision_plugins {
class EigenValsAndVecs : public CornerLineDetection
{
public:
    enum EigenType{MIN_EIGEN_VAL, EIGEN_VALS_AND_VECS};

    EigenValsAndVecs();

    virtual void process();
    virtual void setup();

protected:
    void        update();
    int         k_size_;
    int         block_size_;
    int         border_type_;
    EigenType   eigen_type_;
};
}
#endif // EIGENVALS_AND_VECS_H
