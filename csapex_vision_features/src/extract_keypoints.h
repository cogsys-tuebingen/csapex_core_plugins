#ifndef EXTRACT_FEATURES_H
#define EXTRACT_FEATURES_H

/// COMPONENT
#include <csapex/model/node.h>

/// PROJECT
#include <cslibs_vision/utils/extractor.h>

namespace csapex
{
class ExtractKeypoints : public csapex::Node
{
public:
    ExtractKeypoints();

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

private:
    void update();

private:
    Extractor::Ptr extractor;

    Input* in_img;
    Input* in_mask;
    Output* out_key;

    bool refresh_;
};

}  // namespace csapex

#endif  // EXTRACT_FEATURES_H
