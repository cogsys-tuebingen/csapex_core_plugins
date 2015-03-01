#ifndef STATIC_ROI_H
#define STATIC_ROI_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class StaticRoi : public csapex::Node
{
public:
    StaticRoi();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input*   in_;
    Output* out_;

};

}

#endif // STATIC_ROI_H
