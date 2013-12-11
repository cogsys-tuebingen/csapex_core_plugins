#ifndef RENDER_ROIS_H
#define RENDER_ROIS_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class RenderROIs : public csapex::Node
{
public:
    RenderROIs();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorIn* input_img_;
    ConnectorIn* input_rois_;

    ConnectorOut* output_;
};

}
#endif // RENDER_ROIS_H
