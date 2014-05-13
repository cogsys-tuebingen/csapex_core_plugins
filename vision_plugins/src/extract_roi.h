#ifndef RENDER_ROIS_H
#define RENDER_ROIS_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class ExtractROI : public csapex::Node
{
public:
    ExtractROI();

    virtual void process();
    virtual void setup();

private:
    ConnectorIn* input_img_;
    ConnectorIn* input_roi_;

    ConnectorOut* output_;
};

}
#endif // RENDER_ROIS_H
