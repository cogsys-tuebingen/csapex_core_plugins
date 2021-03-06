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
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    void update();

private:
    Extractor::Ptr extractor;

    Input* in_img;
    Input* in_key;
    Output* out_des;

    bool refresh_;
};

}  // namespace csapex

#endif  // EXTRACT_DESCRIPTORS_H
