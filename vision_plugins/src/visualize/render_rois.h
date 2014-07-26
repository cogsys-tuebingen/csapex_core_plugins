#ifndef RENDER_ROIS_H
#define RENDER_ROIS_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class RenderROIs : public csapex::Node
{
public:
    RenderROIs();

    virtual void setupParameters();
    virtual void process();
    virtual void setup();

private:
    Input* input_img_;
    Input* input_rois_;

    Output* output_;
};

}
#endif // RENDER_ROIS_H
