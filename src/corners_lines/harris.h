#ifndef HARRIS_H
#define HARRIS_H

/// COMPONENT
#include "detection.h"

namespace csapex {
class CornerHarris : public CornerLineDetection
{
public:
    CornerHarris();

    virtual void process();
    virtual void setup();

protected:
    void update();

    double k_;
    int    k_size_;
    int    block_size_;
    int    border_type_;

};

}
#endif // HARRIS_H
