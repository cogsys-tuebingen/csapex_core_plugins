#ifndef CANNY_H
#define CANNY_H

/// COMPONENT
#include "corner_line_detection.h"

namespace csapex {
class Canny : public CornerLineDetection
{
public:
    Canny();
    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    void update();

    double  threshold_1_;
    double  threshold_2_;
    int     aperture_size_;
    bool    L2_gradient_;

};
}


#endif // CANNY_H
