#ifndef VJ_DETECTOR_H
#define VJ_DETECTOR_H

/// PROJECT
#include <csapex/model/node.h>

class CascadeDetector;
class ImageScanner;

namespace csapex {
class VJDetector : public csapex::Node
{
public:
    VJDetector();
    ~VJDetector();

    virtual void process();
    virtual void setup();

private:
    Output* output_;

    Input* input_;

    std::string file_;

    CascadeDetector* vj_detector;
    ImageScanner* image_scanner;
};
}

#endif // VJ_DETECTOR_H
