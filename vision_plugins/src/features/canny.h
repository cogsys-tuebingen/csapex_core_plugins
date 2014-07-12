#ifndef CANNY_H
#define CANNY_H

/// COMPONENT
#include "corner_line_detection.h"

namespace vision_plugins {
class Canny : public CornerLineDetection
{
public:
    Canny();
    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    void update();

    double  threshold_1_;
    double  threshold_2_;
    int     aperture_size_;
    bool    L2_gradient_;

};
}


#endif // CANNY_H
