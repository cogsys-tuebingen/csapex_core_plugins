#ifndef HISTOGRAM_VISION_H
#define HISTOGRAM_VISION_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class Histogram : public csapex::Node
{
public:
    Histogram();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    csapex::Output*   output_;
    csapex::Input*    input_;
    csapex::Input*    mask_;

    int  bins_;
    int  last_type_;
    bool uniform_;
    bool accumulate_;
    bool min_max_;
    bool min_max_global_;
    std::pair<float, float> min_max_value_;

    void update();

    void resetMinMax();
};
}

#endif // HISTOGRAM_VISION_H
