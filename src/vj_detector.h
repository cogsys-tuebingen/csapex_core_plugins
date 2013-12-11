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

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorOut* output_;

    ConnectorIn* input_;

    CascadeDetector* vj_detector;
    ImageScanner* image_scanner;
};
}

#endif // VJ_DETECTOR_H
