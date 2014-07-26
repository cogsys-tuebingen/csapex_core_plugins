#ifndef SCAN_RENDERER_H
#define SCAN_RENDERER_H

/// COMPONENT
#include "renderer.h"

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class ScanRenderer : public csapex::Node
{
public:
    ScanRenderer();

    virtual void setup();
    virtual void process();

    template <typename ScanType>
    void doProcess(ScanType& s);

private:
    Input* input_;
    Output* output_;

    Renderer renderer;
};

}

#endif // SCAN_RENDERER_H
