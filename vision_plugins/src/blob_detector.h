#ifndef BLOB_DETECTOR_H
#define BLOB_DETECTOR_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class BlobDetector : public csapex::Node
{
public:
    BlobDetector();
    ~BlobDetector();

    virtual void process();
    virtual void setup();

private:
    ConnectorOut* output_;
    ConnectorOut* output_debug_;

    ConnectorIn* input_;
};
}

#endif // BLOB_DETECTOR_H
