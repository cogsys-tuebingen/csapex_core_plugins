#ifndef SCAN_RENDERER_H
#define SCAN_RENDERER_H

/// COMPONENT
#include <csapex_scan_2d/renderer.h>

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class ScanRenderer : public csapex::Node
{
public:
    ScanRenderer();

    virtual void setupParameters(Parameterizable& params);
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    template <typename ScanType>
    void doProcess(const ScanType& s);

private:
    Input* input_;
    Output* output_;

    Renderer renderer;
};

}

#endif // SCAN_RENDERER_H
