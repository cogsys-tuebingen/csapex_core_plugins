#ifndef EXTRACT_DESCRIPTORS_H
#define EXTRACT_DESCRIPTORS_H

/// COMPONENT
#include <csapex/model/node.h>

/// PROJECT
#include <cslibs_vision/utils/extractor.h>

namespace csapex
{

class ExtractDescriptors : public csapex::Node
{
public:
    ExtractDescriptors();    

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    void update();

private:
    Extractor::Ptr extractor;

    Input* in_img;
    Input* in_key;
    Output* out_des;

    bool refresh_;
};

}

#endif // EXTRACT_DESCRIPTORS_H
